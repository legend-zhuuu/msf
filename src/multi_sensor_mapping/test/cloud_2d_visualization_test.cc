#include "multi_sensor_mapping/visualizer/lidar_mapper_2d_visualizer.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  // 点云加载
  std::string map_path =
      "/home/hkw/dataset/backpack/yuquan-map/global_cloud.pcd";

  CloudTypePtr map_cloud(new CloudType);
  pcl::io::loadPCDFile(map_path, *map_cloud);

  LidarMapper2DVisualizer visualizer_;
  visualizer_.DisplayGlobalCloud(map_cloud);

  cv::Mat visualization_img;
  if (visualizer_.GetVisulizationImage(visualization_img)) {
    cv::imshow("visualization", visualization_img);
    cv::waitKey();
  }

  return 0;
}