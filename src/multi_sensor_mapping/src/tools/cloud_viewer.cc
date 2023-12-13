#include <ros/ros.h>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_viewer");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  ros::Publisher pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1);

  std::string cloud_path;
  nh_param.param<std::string>("cloud_path", cloud_path, "");

  CloudTypePtr cloud(new CloudType);
  pcl::io::loadPCDFile(cloud_path, *cloud);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";

  ros::Rate rate(0.5);
  while (ros::ok()) {
    pub_cloud.publish(cloud_msg);
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}