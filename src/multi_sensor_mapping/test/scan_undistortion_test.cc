#include "multi_sensor_mapping/frontend/lidar/scan_undistortion.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "config.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/data_manager.h"
#include "multi_sensor_mapping/utils/utils_log.h"

using namespace std;
using namespace multi_sensor_mapping;

/// @brief 激光-IMU的旋转外参
Eigen::Quaterniond lidar_to_imu_rotation;
/// @brief IMU-激光雷达的旋转外参
Eigen::Quaterniond imu_to_lidar_rotation;

void ConvertImuToLidarFrame(IMUData& imu_data) {
  imu_data.gyro = imu_to_lidar_rotation * imu_data.gyro;
  imu_data.accel = imu_to_lidar_rotation * imu_data.accel;
  imu_data.orientation = imu_data.orientation * lidar_to_imu_rotation;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_undistortion_test");
  ros::NodeHandle nh;

  ros::Publisher pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/raw_cloud", 1);
  ros::Publisher pub_undistortion_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/undistortion_cloud", 1);

  std::string config_path = std::string(MSM_SOURCE_DIR) + "/config/PMTD4.0";

  /// 加载参数
  std::shared_ptr<ParamSet> param_set(new ParamSet);
  param_set->Load(config_path);
  param_set->PrintAll();

  Eigen::Quaterniond major_lidar_quater = Eigen::Quaterniond(
      param_set->GetExtrinsicParams()
          ->lidar_to_baselink[param_set->GetSensorParams()->major_lidar_]
          .block<3, 3>(0, 0));
  lidar_to_imu_rotation =
      param_set->GetExtrinsicParams()->imu_to_baselink_rotation_.inverse() *
      major_lidar_quater;
  imu_to_lidar_rotation = lidar_to_imu_rotation.inverse();

  // 读取数据包
  std::string bag_path = "/home/hkw/dataset/PMTD/pmtd_calib_test_01.bag";

  std::shared_ptr<DataConverter> data_converter(new DataConverter);

  std::shared_ptr<ScanUndistortion> scan_undistortion(
      new ScanUndistortion(false, true, false));

  std::shared_ptr<DataManager> data_manager =
      std::make_shared<DataManager>(data_converter);
  data_manager->ReadDataFromRosbag(bag_path, param_set->GetSensorParams());

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  rosbag::View view;
  std::vector<std::string> topics;

  std::string lidar_topic = param_set->GetSensorParams()->MajorLidarTopic();

  topics.push_back(lidar_topic);

  view.addQuery(bag, rosbag::TopicQuery(topics));

  TimedCloudData last_cloud_data;
  TimedCloudData cloud_data;
  for (rosbag::MessageInstance const m : view) {
    const std::string& topic = m.getTopic();

    if (topic == lidar_topic) {
      sensor_msgs::PointCloud2::ConstPtr lidar_msg =
          m.instantiate<sensor_msgs::PointCloud2>();
      cloud_data = data_converter->ConvertVelCloudData(lidar_msg);

      std::vector<IMUData> imu_data_vec;
      if (!last_cloud_data.cloud) {
        last_cloud_data = cloud_data;
      } else {
        if (!data_manager->GetImuDataInterval(imu_data_vec,
                                              last_cloud_data.timestamp,
                                              cloud_data.timestamp)) {
          AWARN_F(
              "[LidarImuMapper] Get IMU wrong , Last cloud time %.3f, current "
              "cloud data %.3f ",
              last_cloud_data.timestamp, cloud_data.timestamp);
        }

        for (size_t i = 0; i < imu_data_vec.size(); i++) {
          ConvertImuToLidarFrame(imu_data_vec[i]);
        }

        if (imu_data_vec.size() > 0) {
          // 畸变矫正
          for (size_t i = 0; i < imu_data_vec.size(); i++) {
            scan_undistortion->InputImuData(imu_data_vec[i]);
          }

          VelRTPointCloudPtr undistorted_cloud(new VelRTPointCloud);
          scan_undistortion->UndistortScan(last_cloud_data.timestamp,
                                           last_cloud_data.cloud,
                                           undistorted_cloud);

          sensor_msgs::PointCloud2 raw_msg;
          pcl::toROSMsg(*last_cloud_data.cloud, raw_msg);
          raw_msg.header.frame_id = "map";

          sensor_msgs::PointCloud2 undistortion_msg;
          pcl::toROSMsg(*undistorted_cloud, undistortion_msg);
          undistortion_msg.header.frame_id = "map";

          pub_cloud.publish(raw_msg);
          pub_undistortion_cloud.publish(undistortion_msg);

          last_cloud_data = cloud_data;

          std::cout << "next frame" << std::endl;
          std::getchar();
          if (!ros::ok()) {
            break;
          }
        }
      }
    }
  }

  return 0;
}
