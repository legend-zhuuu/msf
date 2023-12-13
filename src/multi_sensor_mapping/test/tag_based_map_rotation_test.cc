#include <ros/ros.h>

#include "multi_sensor_mapping/map/grid_map_generator.h"
#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

using namespace multi_sensor_mapping;

bool LoadTag(std::string _tag_info_path,
             std::vector<PoseData>& _pose_data_vec) {
  YAML::Node tag_node;
  try {
    tag_node = YAML::LoadFile(_tag_info_path);
  } catch (...) {
    AERROR_F(
        " [LidarMapSession] The format of config file [ %s ] is wrong, Please "
        "check (e.g. indentation).",
        _tag_info_path.c_str());
    return false;
  }

  for (size_t i = 0; i < tag_node.size(); i++) {
    PoseData tag_pose;
    tag_pose.position = Eigen::Vector3d(tag_node[i]["x"].as<double>(),
                                        tag_node[i]["y"].as<double>(),
                                        tag_node[i]["z"].as<double>());
    tag_pose.orientation = Eigen::Quaterniond(
        tag_node[i]["qw"].as<double>(), tag_node[i]["qx"].as<double>(),
        tag_node[i]["qy"].as<double>(), tag_node[i]["qz"].as<double>());
    _pose_data_vec.push_back(tag_pose);
  }

  std::cout << "tag pose " << _pose_data_vec.size() << std::endl;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tag_based_map_rotation_test");
  ros::NodeHandle nh;

  LidarMapperRosVisualizer visualizer;

  std::string map_path = "/home/hkw/dataset/backpack/shihushan_test1";
  std::string tag_info_path = map_path + "/tag_info.yaml";
  std::string global_map_path = map_path + "/global_cloud.pcd";

  Eigen::Quaterniond rotation_y = Eigen::Quaterniond(
      Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d(0, 1, 0)));

  std::vector<PoseData> tag_pose;
  LoadTag(tag_info_path, tag_pose);

  for (size_t i = 0; i < tag_pose.size(); i++) {
    tag_pose[i].orientation *= rotation_y;
  }
  double yaw, pitch, roll;
  utils::RoationMatrixd2YPR(tag_pose.front().orientation.toRotationMatrix(),
                            yaw, pitch, roll);
  std::cout << "roll : " << roll / M_PI * 180.0 << std::endl;
  std::cout << "pitch : " << pitch / M_PI * 180.0 << std::endl;
  std::cout << "yaw : " << yaw / M_PI * 180.0 << std::endl;

  Eigen::Quaterniond map_rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(-yaw - M_PI / 2, Eigen::Vector3d(0, 0, 1)));

  CloudTypePtr map_cloud(new CloudType);
  pcl::io::loadPCDFile(global_map_path, *map_cloud);

  CloudTypePtr map_transformed(new CloudType);

  pcl::transformPointCloud(*map_cloud, *map_transformed,
                           Eigen::Vector3f::Zero(),
                           map_rotation.template cast<float>());
  for (size_t i = 0; i < tag_pose.size(); i++) {
    tag_pose[i].position = map_rotation * tag_pose[i].position;
    tag_pose[i].orientation = map_rotation * tag_pose[i].orientation;
  }

  std::string grid_map_folder = map_path + "/grid_map";
  if (!boost::filesystem::exists(grid_map_folder)) {
    boost::filesystem::create_directories(grid_map_folder);
  }
  GridMapGenerator global_map_generator(map_transformed, grid_map_folder);
  global_map_generator.GenerateMap("global_map");

  ros::Rate rate(0.5);
  while (ros::ok()) {
    visualizer.DisplayCurrentCloud(map_transformed);
    visualizer.DisplayTagPoses(tag_pose);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}