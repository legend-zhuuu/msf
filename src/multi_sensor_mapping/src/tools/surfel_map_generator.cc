
#include <ros/ros.h>

#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/surfel_octo_map.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "surfel_map_generator");
  ros::NodeHandle nh_param("~");

  // 地图路径
  std::string session_path;
  double leaf_resolution;
  int depth_level;
  int query_min_depth;
  int qurery_max_depth;
  double min_planarity;
  nh_param.param<std::string>("session_path", session_path, "");
  nh_param.param<double>("leaf_resolution", leaf_resolution, 0.1);
  nh_param.param<int>("depth_level", depth_level, 16);
  nh_param.param<int>("query_min_depth", query_min_depth, 0);
  nh_param.param<int>("qurery_max_depth", qurery_max_depth, 16);
  nh_param.param<double>("min_planarity", min_planarity, 0.9);

  // 地图任务
  std::shared_ptr<LidarMapSession> session =
      std::make_shared<LidarMapSession>();
  session->Load(session_path);

  std::shared_ptr<SurfelOctoMap> surfel_map =
      std::make_shared<SurfelOctoMap>(leaf_resolution, depth_level);

  LidarMapperRosVisualizer visualizer;

  // 地图生成
  int unit_size = session->GetMapUnitSize();
  for (int i = 0; i < unit_size; i++) {
    CloudTypePtr cloud = session->GetMapUnitFullCloud(i);
    Eigen::Matrix4d pose = session->GetMapUnitPose(i);
    CloudTypePtr cloud_in_map(new CloudType);
    pcl::transformPointCloud(*cloud, *cloud_in_map, pose);

    surfel_map->InsertCloud(cloud_in_map);
  }

  CloudTypePtr global_cloud = session->GetGlobalMap();

  // 地图可视化
  std::vector<SurfelUnit> surfel_units = surfel_map->QuerySurfelUnits(
      query_min_depth, qurery_max_depth, min_planarity);

  ros::Rate rate(0.5);
  while (ros::ok()) {
    visualizer.DisplaySurfelMap(surfel_units);
    visualizer.DisplayGlobalCloud(global_cloud);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}