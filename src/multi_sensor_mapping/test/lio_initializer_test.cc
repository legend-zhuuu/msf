#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "config.h"
#include "multi_sensor_mapping/frontend/imu/imu_initializer.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/data_manager.h"
#include "multi_sensor_mapping/utils/rslidar_decoder.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"

using namespace multi_sensor_mapping;

bool StaticInitializationProcess(
    const std::shared_ptr<DataManager>& _data_manager,
    Eigen::Quaterniond& _imu_start_rotation, double _start_time) {
  ImuInitializer imu_initializer(9.81, 0.75, 0.35);
  int imu_count = 0;
  while (true) {
    IMUData imu_data;
    if (_data_manager->GetImuData(imu_data, imu_count)) {
      imu_initializer.FeedImuData(imu_data);
      imu_count++;
    } else {
      break;
    }

    // IMU初始化检测
    if (imu_count % 50 == 0) {
      if (imu_initializer.CheckImuStaticState(_start_time)) {
        // IMU初始化成功
        _imu_start_rotation = imu_initializer.GetI0ToG();

        return true;
      }
    }
  }

  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_initializer_test");
  ros::NodeHandle nh;

  ros::Publisher pub_lidar =
      nh.advertise<sensor_msgs::PointCloud2>("/lidar_cloud", 1);

  std::string bag_path =
      "/home/hkw/DATA/Dataset/MappingBackpackV3/ZJUT/zjut3.bag";
  std::string config_name = "mapping_backpack_v3";
  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/" + config_name;

  std::shared_ptr<ParamSet> param_set(new ParamSet);
  if (!param_set->Load(param_set_path)) {
    return 0;
  }

  SensorParams::Ptr sensor_params = param_set->GetSensorParams();
  ExtrinsicParams::Ptr extrinsic_params = param_set->GetExtrinsicParams();

  std::shared_ptr<DataConverter> data_converter_ =
      std::make_shared<DataConverter>(0, 100);
  std::shared_ptr<DataManager> data_manager =
      std::make_shared<DataManager>(data_converter_);

  if (!data_manager->ReadDataFromRosbag(bag_path, sensor_params)) {
    return false;
  }

  Eigen::Quaterniond imu_start_rotation;
  double start_time;
  StaticInitializationProcess(data_manager, imu_start_rotation, start_time);

  Eigen::Quaterniond major_lidar_quater = Eigen::Quaterniond(
      extrinsic_params->lidar_to_baselink[sensor_params->major_lidar_]
          .block<3, 3>(0, 0));
  Eigen::Quaterniond lidar_to_imu_rotation =
      extrinsic_params->imu_to_baselink_rotation_.inverse() *
      major_lidar_quater;

  std::shared_ptr<RslidarDecoder> rslidar_decoder_ptr;
  if (sensor_params->MajorLidarFormat() >=
      LidarDataFormat::RS_PACKET_RSHELIOS_16P) {
    rslidar_decoder_ptr =
        std::make_shared<RslidarDecoder>(sensor_params->MajorLidarFormat());
  }

  // Step7 读取数据包，激光里程计
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  rosbag::View view;
  std::vector<std::string> topics;
  // TODO 当前版本只支持单激光建图
  topics.push_back(sensor_params->MajorLidarTopic());

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  ros::Time time_finish = view_full.getEndTime();
  ros::Duration delta_durr = ros::Duration(0.1);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init - delta_durr,
                time_finish + delta_durr);

  size_t process_lidar_cnt = 0;

  TimedCloudData last_cloud_data;
  TimedCloudData cloud_data;
  bool cloud_ready_flag = false;
  for (rosbag::MessageInstance const m : view) {
    const std::string& topic = m.getTopic();
    if (sensor_params->MajorLidarTopic() == topic) {
      std::string data_type = m.getDataType();
      if (data_type == std::string("sensor_msgs/PointCloud2")) {
        /// PointCloud2 数据处理
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        if (sensor_params->MajorLidarFormat() == LidarDataFormat::RSLIDAR) {
          cloud_data = data_converter_->ConvertRsCloudData(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params->MajorLidarFormat() ==
                   LidarDataFormat::RSLIDAR_LEGACY) {
          cloud_data = data_converter_->ConvertLegacyRsCloudData(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params->MajorLidarFormat() ==
                   LidarDataFormat::VELODYNE) {
          cloud_data = data_converter_->ConvertVelCloudData(lidar_msg);
          cloud_ready_flag = true;
        }

      } else if (data_type == "rslidar_msg/RslidarPacket") {
        /// RslidarPacket 数据处理
        rslidar_msg::RslidarPacket::ConstPtr packet_msg =
            m.instantiate<rslidar_msg::RslidarPacket>();

        if (rslidar_decoder_ptr->DecodePacket(*packet_msg)) {
          std::shared_ptr<RsPointCloudMsg> cloud =
              rslidar_decoder_ptr->GetCloud();
          RsRTPointCloudPtr raw_cloud(new RsRTPointCloud);
          *raw_cloud = *cloud;

          data_converter_->PreprocessCloud(raw_cloud, cloud_data.cloud,
                                           cloud_data.timestamp);
          // AINFO_F("[LidarImuMapper] Current  cloud data %.3f ",
          //         cloud_data.timestamp);
          cloud_ready_flag = true;
        }
      }
      process_lidar_cnt++;
    }

    if (!cloud_ready_flag) {
      continue;
    }

    cloud_ready_flag = false;

    if (cloud_data.timestamp >= start_time) {
      break;
    }
  }

  VelRTPointCloudPtr cloud_in_global(new VelRTPointCloud);
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

  Eigen::Quaterniond init_rotation = imu_start_rotation * lidar_to_imu_rotation;
  transform_matrix.block<3, 3>(0, 0) =
      init_rotation.toRotationMatrix().template cast<float>();

  pcl::transformPointCloud(*cloud_data.cloud, *cloud_in_global,
                           transform_matrix);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_in_global, cloud_msg);
  cloud_msg.header.frame_id = "map";

  ros::Rate rate(1);
  while (ros::ok()) {
    pub_lidar.publish(cloud_msg);
    rate.sleep();
  }

  return 0;
}