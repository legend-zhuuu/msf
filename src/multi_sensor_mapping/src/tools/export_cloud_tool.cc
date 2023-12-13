#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "export_cloud_tool");
  ros::NodeHandle nh_param("~");

  std::string session_path;
  std::string export_path;
  double cloud_resolution;

  nh_param.param<std::string>("session_path", session_path, "");
  nh_param.param<std::string>("export_path", export_path, "");
  nh_param.param<double>("cloud_resolution", cloud_resolution, 0.01);

  // 加载地图
  LidarMapSession session;
  session.Load(session_path);

  CloudTypePtr map_cloud = session.GetGlobalMap(cloud_resolution);

  std::cout << "Export Cloud success " << std::endl;

  if (!boost::filesystem::exists(export_path)) {
    boost::filesystem::create_directories(export_path);
  }

  std::string cloud_path = export_path + "/map_cloud.pcd";
  pcl::io::savePCDFileBinaryCompressed(cloud_path, *map_cloud);

  std::cout << "Save Cloud success " << std::endl;

  return 0;
}