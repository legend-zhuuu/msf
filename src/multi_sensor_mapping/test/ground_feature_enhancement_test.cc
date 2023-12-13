#include "multi_sensor_mapping/map/grid_map_generator.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  std::string cloud_path =
      "/home/nical/BDCA/dataset/"
      "GongyuanBuildingParkingLot(20230315)-Refined-20230316/ground_cloud.pcd";

  GridMapGenerator lidar_image(cloud_path, 0.01);

  lidar_image.GenerateGroundFeatureMap();

  cv::imwrite("visual_clud.jpeg", lidar_image.grey_image_);

  return 0;
}