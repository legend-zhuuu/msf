#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "multi_sensor_mapping/frontend/lidar/feature_extractor.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/rslidar_decoder.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "feature_extractor_viewer");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  ros::Publisher pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1);
  ros::Publisher pub_corner_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("corner_cloud", 1);
  ros::Publisher pub_surface_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("surface_cloud", 1);

  std::string bag_path;
  std::string lidar_topic;

  nh_param.param<std::string>("bag_path", bag_path, "");
  nh_param.param<std::string>("lidar_topic", lidar_topic, "/rslidar_points");

  int num_ring = 32;
  int num_horizon = 1800;
  double edge_threshold = 1.0;
  double surf_threshold = 0.1;
  double surf_feature_leaf_size = 0.2;

  std::shared_ptr<DataConverter> data_converter_ =
      std::make_shared<DataConverter>();

  std::shared_ptr<FeatureExtractor> feature_extractor_ptr_ =
      std::make_shared<FeatureExtractor>(num_ring, num_horizon, edge_threshold,
                                         surf_threshold,
                                         surf_feature_leaf_size);

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  rosbag::View view;
  std::vector<std::string> topics;

  topics.push_back(lidar_topic);

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  ros::Time time_finish = view_full.getEndTime();
  ros::Duration delta_durr = ros::Duration(0.1);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init - delta_durr,
                time_finish + delta_durr);

  TimedCloudData cloud_data;
  bool cloud_ready_flag = false;
  for (rosbag::MessageInstance const m : view) {
    const std::string& topic = m.getTopic();
    if (lidar_topic == topic) {
      std::string data_type = m.getDataType();
      if (data_type == std::string("sensor_msgs/PointCloud2")) {
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        cloud_data = data_converter_->ConvertRsCloudData(lidar_msg);
        cloud_ready_flag = true;
      } else if (data_type == "rslidar_msg/RslidarPacket") {
        rslidar_msg::RslidarPacket::ConstPtr packet_msg =
            m.instantiate<rslidar_msg::RslidarPacket>();

        /// TODO
      }
    }

    if (!cloud_ready_flag) {
      continue;
    }

    cloud_ready_flag = false;

    CloudTypePtr corner_cloud(new CloudType);
    CloudTypePtr surface_cloud(new CloudType);
    // 特征提取
    feature_extractor_ptr_->ExtractLOAMFeature(cloud_data.cloud, corner_cloud,
                                               surface_cloud);

    // 点云显示
    sensor_msgs::PointCloud2 raw_cloud_msg;
    sensor_msgs::PointCloud2 corner_cloud_msg;
    sensor_msgs::PointCloud2 surface_cloud_msg;

    pcl::toROSMsg(*cloud_data.cloud, raw_cloud_msg);
    pcl::toROSMsg(*corner_cloud, corner_cloud_msg);
    pcl::toROSMsg(*surface_cloud, surface_cloud_msg);

    raw_cloud_msg.header.frame_id = "map";
    corner_cloud_msg.header.frame_id = "map";
    surface_cloud_msg.header.frame_id = "map";

    pub_cloud.publish(raw_cloud_msg);
    pub_corner_cloud.publish(corner_cloud_msg);
    pub_surface_cloud.publish(surface_cloud_msg);

    std::getchar();

    if (!ros::ok()) {
      break;
    }
  }

  return 0;
}