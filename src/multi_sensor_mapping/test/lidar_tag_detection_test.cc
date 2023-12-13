#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "multi_sensor_mapping/frontend/lidar/lidar_tag_detector.h"
#include "multi_sensor_mapping/utils/data_converter.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_tag_detection_test");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  ros::Publisher pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1);
  ros::Publisher pub_cloud_inner =
      nh.advertise<sensor_msgs::PointCloud2>("inner_cloud", 1);

  ros::Publisher pub_roi_marker =
      nh.advertise<visualization_msgs::MarkerArray>("roi_marker", 1);
  ros::Publisher pub_tag_poses =
      nh.advertise<geometry_msgs::PoseArray>("tag_pose", 1);

  ros::Publisher pub_edge_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("edge_cloud", 10);
  ros::Publisher pub_filtered_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

  std::string bag_path;
  std::string lidar_topic;
  nh_param.param<std::string>("bag_path", bag_path, "");
  nh_param.param<std::string>("lidar_topic", lidar_topic, "/rslidar_points");

  std::shared_ptr<DataConverter> data_converter_ =
      std::make_shared<DataConverter>();

  std::shared_ptr<LidarTagDetector> lidar_tag_detector_ =
      std::make_shared<LidarTagDetector>();

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
  ros::Rate rate(10);
  for (rosbag::MessageInstance const m : view) {
    const std::string& topic = m.getTopic();
    if (lidar_topic == topic) {
      std::string data_type = m.getDataType();
      if (data_type == std::string("sensor_msgs/PointCloud2")) {
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        cloud_data = data_converter_->ConvertRsCloudData(lidar_msg);
        cloud_ready_flag = true;
      }
    }

    if (!cloud_ready_flag) {
      continue;
    }

    lidar_tag_detector_->DetectTag(cloud_data.cloud);

    std::vector<ROIArea> roi_area = lidar_tag_detector_->GetROIArea();
    std::cout << roi_area.size() << std::endl;

    cloud_ready_flag = false;

    auto tag_poses = lidar_tag_detector_->GetTagPose();
    geometry_msgs::PoseArray tag_pose_msg;
    tag_pose_msg.header.frame_id = "map";
    for (auto tag_pose : tag_poses) {
      geometry_msgs::Pose pose_msg;
      pose_msg.position.x = tag_pose.position(0);
      pose_msg.position.y = tag_pose.position(1);
      pose_msg.position.z = tag_pose.position(2);

      pose_msg.orientation.w = tag_pose.orientation.w();
      pose_msg.orientation.x = tag_pose.orientation.x();
      pose_msg.orientation.y = tag_pose.orientation.y();
      pose_msg.orientation.z = tag_pose.orientation.z();
      tag_pose_msg.poses.push_back(pose_msg);
    }
    pub_tag_poses.publish(tag_pose_msg);

    // 点云显示
    sensor_msgs::PointCloud2 raw_cloud_msg;
    pcl::toROSMsg(*cloud_data.cloud, raw_cloud_msg);
    raw_cloud_msg.header.frame_id = "map";
    pub_cloud.publish(raw_cloud_msg);

    CloudType inner_cloud;
    for (size_t i = 0; i < roi_area.size(); ++i) {
      inner_cloud += *(roi_area[i].inner_cloud);
    }
    sensor_msgs::PointCloud2 inner_cloud_msg;
    pcl::toROSMsg(inner_cloud, inner_cloud_msg);
    inner_cloud_msg.header.frame_id = "map";
    pub_cloud_inner.publish(inner_cloud_msg);

    // ROI marker 显示
    visualization_msgs::MarkerArray roi_marker_array;
    for (size_t i = 0; i < roi_area.size(); ++i) {
      Eigen::Vector3f center =
          (roi_area[i].min_boundary + roi_area[i].max_boundary) / 2;
      Eigen::Vector3f length =
          roi_area[i].max_boundary - roi_area[i].min_boundary;
      visualization_msgs::Marker roi_marker;
      roi_marker.header.frame_id = "map";
      roi_marker.header.stamp = ros::Time::now();
      roi_marker.ns = "roi_marker";
      roi_marker.id = i;
      roi_marker.type = visualization_msgs::Marker::CUBE;
      roi_marker.action = visualization_msgs::Marker::ADD;
      roi_marker.pose.position.x = center(0);
      roi_marker.pose.position.y = center(1);
      roi_marker.pose.position.z = center(2);
      roi_marker.pose.orientation.w = 1;
      roi_marker.scale.x = length(0);
      roi_marker.scale.y = length(1);
      roi_marker.scale.z = length(2);
      roi_marker.color.r = 0.0;
      roi_marker.color.g = 1.0;
      roi_marker.color.b = 0.0;
      roi_marker.color.a = 0.5;

      roi_marker_array.markers.push_back(roi_marker);
    }

    pub_roi_marker.publish(roi_marker_array);

    CloudType tag_cloud = *(lidar_tag_detector_->GetEdgecloud());
    sensor_msgs::PointCloud2 tag_cloud_msg;
    pcl::toROSMsg(tag_cloud, tag_cloud_msg);
    tag_cloud_msg.header.frame_id = "map";
    pub_edge_cloud.publish(tag_cloud_msg);

    CloudType filtered_cloud = *(lidar_tag_detector_->GetFilteredCloud());
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "map";
    pub_filtered_cloud.publish(filtered_cloud_msg);

    std::getchar();
    if (!ros::ok()) {
      break;
    } else {
      rate.sleep();
    }
  }
}
