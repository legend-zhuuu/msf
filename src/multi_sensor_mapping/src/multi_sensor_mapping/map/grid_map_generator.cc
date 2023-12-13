#include "multi_sensor_mapping/map/grid_map_generator.h"

#include <boost/filesystem.hpp>

using namespace cv;

namespace multi_sensor_mapping {

GridMapGenerator::GridMapGenerator(std::string _cloud_path,
                                   double _grid_resolution)
    : grid_resolution_(_grid_resolution), global_cloud_(new CloudType) {
  boost::filesystem::path p(_cloud_path);
  if (p.extension() != ".pcd") {
    std::cout << RED << "Cloud path is wrong : " << _cloud_path << RESET
              << std::endl;
    return;
  }

  cache_path_ = p.parent_path().string();

  pcl::io::loadPCDFile(_cloud_path, *global_cloud_);
}

GridMapGenerator::GridMapGenerator(CloudTypePtr _cloud,
                                   std::string _session_path,
                                   double _grid_resolution)
    : grid_resolution_(_grid_resolution) {
  cache_path_ = _session_path;
  global_cloud_ = _cloud;
}

void GridMapGenerator::SetKeyFramePoses(
    const std::vector<Eigen::Vector3d>& _key_frame_poses) {
  key_frame_poses_ = _key_frame_poses;
}

void GridMapGenerator::Cloud2GreyImage() {
  Eigen::Vector4f min_cloud;
  Eigen::Vector4f max_cloud;
  global_cloud_->is_dense = false;

  pcl::getMinMax3D<PointType>(*global_cloud_, min_cloud, max_cloud);
  min_cloud(0) -= 1;
  min_cloud(1) -= 1;
  max_cloud(0) += 1;
  max_cloud(1) += 1;
  float len_x = max_cloud(0) - min_cloud(0);
  float len_y = max_cloud(1) - min_cloud(1);

  origin_position_ = Eigen::Vector3d(min_cloud(0), min_cloud(1), 0);

  // x轴方向是width y方向是height
  int width = ((int)(len_x / grid_resolution_));
  int height = ((int)(len_y / grid_resolution_));

  grey_image_ = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));

  for (size_t i = 0; i < global_cloud_->size(); i++) {
    int index_y =
        int((global_cloud_->points[i].x - min_cloud(0)) / grid_resolution_);
    int index_x = height - int((global_cloud_->points[i].y - min_cloud(1)) /
                               grid_resolution_);

    if ((int)grey_image_.at<uchar>(index_x, index_y) >= 50)
      grey_image_.at<uchar>(index_x, index_y) -= 50;
  }

  cv::normalize(grey_image_, grey_image_, 0, 255, cv::NORM_MINMAX);

  if (!key_frame_poses_.empty()) {
    grey_image_.copyTo(road_map_);

    for (size_t i = 1; i < key_frame_poses_.size(); i++) {
      int index_y_start =
          int((key_frame_poses_[i - 1](0) - min_cloud(0)) / grid_resolution_);
      int index_x_start =
          height -
          int((key_frame_poses_[i - 1](1) - min_cloud(1)) / grid_resolution_);

      int index_y_end =
          int((key_frame_poses_[i](0) - min_cloud(0)) / grid_resolution_);
      int index_x_end = height - int((key_frame_poses_[i](1) - min_cloud(1)) /
                                     grid_resolution_);

      cv::line(road_map_, cv::Point(index_y_start, index_x_start),
               cv::Point(index_y_end, index_x_end), cv::Scalar(0), 2, CV_AA);
    }
  }
}

void GridMapGenerator::GenerateGroundFeatureMap() {
  Eigen::Vector4f min_cloud;
  Eigen::Vector4f max_cloud;
  global_cloud_->is_dense = false;

  pcl::getMinMax3D<PointType>(*global_cloud_, min_cloud, max_cloud);
  min_cloud(0) -= 1;
  min_cloud(1) -= 1;
  max_cloud(0) += 1;
  max_cloud(1) += 1;
  float len_x = max_cloud(0) - min_cloud(0);
  float len_y = max_cloud(1) - min_cloud(1);

  origin_position_ = Eigen::Vector3d(min_cloud(0), min_cloud(1), 0);

  // x轴方向是width y方向是height
  int width = ((int)(len_x / grid_resolution_));
  int height = ((int)(len_y / grid_resolution_));

  grey_image_ = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
  std::vector<int> image_points(height * width, 0);

#pragma omp parallel for num_threads(8)
  for (size_t i = 0; i < global_cloud_->size(); i++) {
    int index_y =
        int((global_cloud_->points[i].x - min_cloud(0)) / grid_resolution_);
    int index_x = height - int((global_cloud_->points[i].y - min_cloud(1)) /
                               grid_resolution_);

    // 均值化
    image_points[index_y * width + index_x]++;
    grey_image_.at<uchar>(index_x, index_y) +=
        (global_cloud_->points[i].intensity -
         grey_image_.at<uchar>(index_x, index_y)) /
        image_points[index_y * width + index_x];
  }

  // 归一化 + 二值化
  cv::normalize(grey_image_, grey_image_, 0, 255, cv::NORM_MINMAX);
  cv::threshold(grey_image_, grey_image_, 25, 255, CV_THRESH_BINARY);
}

void GridMapGenerator::DilateCallback(int, void* data) {
  GridMapGenerator* p = (GridMapGenerator*)data;
  p->Cloud2GreyImage();

  cv::Mat visual_image;
  if (p->dilate_size_ < 0) {
    visual_image = p->grey_image_.clone();
  } else {
    Mat element = getStructuringElement(
        MORPH_RECT, Size(2 * p->dilate_size_ + 1, 2 * p->dilate_size_ + 1),
        Point(p->dilate_size_, p->dilate_size_));

    cv::erode(p->grey_image_, visual_image, element);
  }

  imshow("Grid Map", visual_image);

  p->SaveGridMap(visual_image);
}

void GridMapGenerator::GenerateMap() {
  /// 创建显示窗口
  namedWindow("Grid Map", CV_WINDOW_NORMAL);

  cvResizeWindow("Grid Map", 400, 800);

  createTrackbar("Kernel size:\n 2n +1", "Grid Map", &this->dilate_size_, 21,
                 DilateCallback, this);

  DilateCallback(0, this);

  waitKey(0);
}

void GridMapGenerator::GenerateMap(std::string _map_name) {
  Cloud2GreyImage();
  SaveGridMap(_map_name, grey_image_);
}

void GridMapGenerator::SaveGridMap(cv::Mat& _image) {
  std::string grey_image_local_name = "map_image.pgm";
  std::string grey_image_path = cache_path_ + "/" + grey_image_local_name;

  std::string grey_image_pgn_path = cache_path_ + "/map_image.png";
  imwrite(grey_image_path, _image);
  imwrite(grey_image_pgn_path, _image);

  std::string yaml_path = cache_path_ + "/map.yaml";
  FILE* yaml = fopen(yaml_path.c_str(), "w");

  fprintf(yaml,
          "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: "
          "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          grey_image_local_name.c_str(), grid_resolution_, origin_position_(0),
          origin_position_(1), origin_position_(2));

  fclose(yaml);
}

void GridMapGenerator::SaveGridMap(std::string _map_name, cv::Mat& _image) {
  std::string grey_image_local_name = _map_name + ".pgm";
  std::string grey_image_path = cache_path_ + "/" + grey_image_local_name;

  std::string grey_image_pgn_path = cache_path_ + "/" + _map_name + ".png";
  imwrite(grey_image_path, _image);
  imwrite(grey_image_pgn_path, _image);

  if (!key_frame_poses_.empty()) {
    std::string road_map_path = cache_path_ + "/" + _map_name + "_road_map.png";
    imwrite(road_map_path, road_map_);
  }

  std::string yaml_path = cache_path_ + "/" + _map_name + "_map.yaml";
  FILE* yaml = fopen(yaml_path.c_str(), "w");

  fprintf(yaml,
          "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: "
          "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          grey_image_local_name.c_str(), grid_resolution_, origin_position_(0),
          origin_position_(1), origin_position_(2));

  fclose(yaml);
}

}  // namespace multi_sensor_mapping
