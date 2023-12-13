#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "config.h"
#include "msm_sdk/msm_sdk.h"
#include "multi_sensor_mapping/utils/utils_tictoc.h"

using namespace msm_sdk;

/// @brief 点云发布器
ros::Publisher pub_cloud;
/// @brief 轨迹发布器
ros::Publisher pub_path;

nav_msgs::Path path_msg;

void PutLidarPose(const StampedPose& _pose) {
  geometry_msgs::PoseStamped path_point;
  path_point.pose.position.x = _pose.position(0);
  path_point.pose.position.y = _pose.position(1);
  path_point.pose.position.z = _pose.position(2);
  path_point.pose.orientation.w = _pose.orientation.w();
  path_point.pose.orientation.x = _pose.orientation.x();
  path_point.pose.orientation.y = _pose.orientation.y();
  path_point.pose.orientation.z = _pose.orientation.z();

  path_msg.poses.push_back(path_point);
  path_msg.header.frame_id = "map";
  pub_path.publish(path_msg);
}

void PutFrameCloud(const StampedCloud& _cloud) {
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(_cloud.cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = "map";

  pub_cloud.publish(pointcloud_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_lidar_imu_odometry");
  ros::NodeHandle nh;

  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("current_cloud", 1);
  pub_path = nh.advertise<nav_msgs::Path>("lidar_path", 1);

  auto msm_version = msm_sdk::GetVersion();
  std::cout << "MSM SDK Version : " << msm_version.toString() << std::endl;

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/mapping_backpack_v2";

  APILidarImuOdometry mapper;
  // 初始化
  mapper.Init(param_set_path);

  // 注册回调函数
  mapper.RegLidarPoseCallback(std::bind(PutLidarPose, std::placeholders::_1));
  mapper.RegFrameCloudCallback(std::bind(PutFrameCloud, std::placeholders::_1));

  mapper.Start();
  ros::spin();
  mapper.Stop();

  return 0;
}