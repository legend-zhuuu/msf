#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "multi_sensor_mapping/frontend/lidar/gicp_scan_matcher.h"
#include "multi_sensor_mapping/frontend/lidar/ndt_scan_matcher.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

using namespace multi_sensor_mapping;

Eigen::Matrix4f ICPRegistration(CloudTypePtr& _target_cloud,
                                CloudTypePtr& _source_cloud,
                                Eigen::Matrix4f _init_mat) {
  utils::DownsampleCloud(*_target_cloud, 0.2);
  utils::DownsampleCloud(*_source_cloud, 0.2);

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(150);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputTarget(_target_cloud);
  icp.setInputSource(_source_cloud);

  CloudType final_cloud;
  icp.align(final_cloud, _init_mat);

  std::cout << "Score : " << icp.getFitnessScore() << std::endl;

  return icp.getFinalTransformation();
}

Eigen::Matrix4f NdtRegistration(CloudTypePtr& _target_cloud,
                                CloudTypePtr& _source_cloud,
                                Eigen::Matrix4f _init_mat) {
  NdtScanMatcher matcher(1.0, 0.5, 4);
  matcher.SetTargetCloud(_target_cloud);
  Eigen::Matrix4f final_transf;
  matcher.Match(_source_cloud, _init_mat, final_transf);
  std::cout << "Score : " << matcher.GetMatchScore() << std::endl;
  return final_transf;
}

Eigen::Matrix4f GICPRegistration(CloudTypePtr& _target_cloud,
                                 CloudTypePtr& _source_cloud,
                                 Eigen::Matrix4f _init_mat) {
  GICPScanMatcher matcher(0.1);
  matcher.SetTargetCloud(_target_cloud);
  Eigen::Matrix4f final_transf;
  matcher.Match(_source_cloud, _init_mat, final_transf);
  std::cout << "Score : " << matcher.GetMatchScore() << std::endl;
  return final_transf;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_icp_registation_test");
  ros::NodeHandle nh;

  ros::Publisher pub_target_cloud_ =
      nh.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);
  ros::Publisher pub_source_cloud_ =
      nh.advertise<sensor_msgs::PointCloud2>("/source_cloud", 1);

  std::string target_cloud_path =
      "/home/hkw/DATA/Dataset/MappingBackpackV2/它人机器人/"
      "HaizhiCenterParkinglot(20230308)/loop_closure/lc-0/"
      "raw_target_cloud.pcd";
  std::string source_cloud_path =
      "/home/hkw/DATA/Dataset/MappingBackpackV2/它人机器人/"
      "HaizhiCenterParkinglot(20230308)/loop_closure/lc-0/raw_source_cloud.pcd";

  Eigen::Matrix4f init_mat;
  init_mat << -0.464861, -0.882878, -0.0665624, -0.251763, 0.885229, -0.46487,
      -0.0162997, 4.22414, -0.0165522, -0.0665001, 0.997649, 0.0707258, 0, 0, 0,
      1;

  CloudTypePtr target_cloud(new CloudType);
  CloudTypePtr source_cloud(new CloudType);

  pcl::io::loadPCDFile(target_cloud_path, *target_cloud);
  pcl::io::loadPCDFile(source_cloud_path, *source_cloud);

  Eigen::Matrix4f icp_result =
      GICPRegistration(target_cloud, source_cloud, init_mat);

  std::cout << icp_result << std::endl;

  sensor_msgs::PointCloud2 target_msg;
  pcl::toROSMsg(*target_cloud, target_msg);
  target_msg.header.frame_id = "map";

  CloudType transformed_source_cloud;
  pcl::transformPointCloud(*source_cloud, transformed_source_cloud, icp_result);
  sensor_msgs::PointCloud2 source_msg;
  pcl::toROSMsg(transformed_source_cloud, source_msg);
  source_msg.header.frame_id = "map";

  ros::Rate rate(1);
  while (ros::ok()) {
    pub_target_cloud_.publish(target_msg);
    pub_source_cloud_.publish(source_msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}