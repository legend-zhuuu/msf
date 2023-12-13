#include "multi_sensor_mapping/visualizer/lidar_mapper_2d_visualizer.h"

#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

LidarMapper2DVisualizer::LidarMapper2DVisualizer()
    : output_img_width_(860),
      output_img_height_(600),
      resolution_(0.2),
      visualization_flag_(false),
      refresh_cloud_map_flag_(true),
      vis_origin_position_(Eigen::Vector3d(0, 0, 0)),
      map_cloud_(new CloudType) {
  UpdateBoundaryParams();
}

void LidarMapper2DVisualizer::SetMapResolution(float _resolution) {
  if (_resolution == resolution_) return;

  if (_resolution < 0.1) {
    resolution_ = 0.1;
  } else {
    resolution_ = _resolution;
  }

  refresh_cloud_map_flag_ = true;
}

bool LidarMapper2DVisualizer::PoseToPixelCoordinate(
    const Eigen::Vector3d& _position, int& _pix_x, int& _pix_y) {
  if (!visualization_flag_) {
    return false;
  }

  if (_position(0) <= map_boundary_x_(0) ||
      _position(0) >= map_boundary_x_(1) ||
      _position(1) <= map_boundary_y_(0) ||
      _position(1) >= map_boundary_y_(1)) {
    return false;
  }

  _pix_y = int((_position(0) - map_boundary_x_(0)) / resolution_);
  _pix_x = output_img_height_ -
           int((_position(1) - map_boundary_y_(0)) / resolution_);
  return true;
}

bool LidarMapper2DVisualizer::PoseToPixelCoordinate(
    const Eigen::Vector3d& _position, const Eigen::Quaterniond& _rotation,
    int& _pix_x, int& _pix_y, float& _angle) {
  if (!visualization_flag_) {
    return false;
  }

  _pix_y = int((_position(0) - map_boundary_x_(0)) / resolution_);
  _pix_x = output_img_height_ -
           int((_position(1) - map_boundary_y_(0)) / resolution_);

  double roll, pitch, yaw;
  utils::RoationMatrixd2YPR(_rotation.toRotationMatrix(), yaw, pitch, roll);
  _angle = yaw * 180.0 / M_PI + 90.0;
  return true;
}

void LidarMapper2DVisualizer::DisplayCloud(
    const CloudTypePtr& _cloud, const Eigen::Vector3d& _postion,
    const Eigen::Quaterniond& _rotation) {
  CloudType cloud_in_map_frame;
  pcl::transformPointCloud(*_cloud, cloud_in_map_frame, _postion, _rotation);
  *map_cloud_ += cloud_in_map_frame;

  // 更新地图
  if (refresh_cloud_map_flag_) {
    vis_origin_position_ = _postion;
    UpdateBoundaryParams();

    vis_map_.create(output_img_height_, output_img_width_, CV_8UC1);
    vis_map_ = cv::Scalar(255);
    // vis_map_ = cv::Mat(output_img_height_, output_img_width_, CV_8UC1,
    //                    cv::Scalar(255));

    for (size_t i = 0; i < map_cloud_->size(); i++) {
      if (map_cloud_->points[i].x < map_boundary_x_(0) ||
          map_cloud_->points[i].x > map_boundary_x_(1) ||
          map_cloud_->points[i].y < map_boundary_y_(0) ||
          map_cloud_->points[i].y > map_boundary_y_(1)) {
        continue;
      }

      int index_y =
          int((map_cloud_->points[i].x - map_boundary_x_(0)) / resolution_);
      int index_x =
          output_img_height_ -
          int((map_cloud_->points[i].y - map_boundary_y_(0)) / resolution_);
      if ((int)vis_map_.at<uchar>(index_x, index_y) >= 10) {
        vis_map_.at<uchar>(index_x, index_y) -= 10;
      }
    }

    cv::normalize(vis_map_, vis_map_, 0, 255, cv::NORM_MINMAX);

    refresh_cloud_map_flag_ = false;
  } else {
    // 位姿态判断
    if (_postion(0) < refresh_map_x_(0) || _postion(0) > refresh_map_x_(1) ||
        _postion(1) < refresh_map_y_(0) || _postion(1) > refresh_map_y_(1)) {
      refresh_cloud_map_flag_ = true;
    }

    // 点云2维渲染
    for (size_t i = 0; i < cloud_in_map_frame.size(); i++) {
      if (cloud_in_map_frame.points[i].x < map_boundary_x_(0) ||
          cloud_in_map_frame.points[i].x > map_boundary_x_(1) ||
          cloud_in_map_frame.points[i].y < map_boundary_y_(0) ||
          cloud_in_map_frame.points[i].y > map_boundary_y_(1)) {
        continue;
      }

      int index_y = int((cloud_in_map_frame.points[i].x - map_boundary_x_(0)) /
                        resolution_);
      int index_x = output_img_height_ -
                    int((cloud_in_map_frame.points[i].y - map_boundary_y_(0)) /
                        resolution_);

      if ((int)vis_map_.at<uchar>(index_x, index_y) >= 50)
        vis_map_.at<uchar>(index_x, index_y) -= 10;
    }

    // cv::normalize(vis_map_, vis_map_, 0, 255, cv::NORM_MINMAX);
  }

  visualization_flag_ = true;
}

void LidarMapper2DVisualizer::DisplayGlobalCloud(const CloudTypePtr& _cloud) {
  *map_cloud_ = *_cloud;
  Eigen::Vector4f min_cloud;
  Eigen::Vector4f max_cloud;
  map_cloud_->is_dense = false;

  pcl::getMinMax3D<PointType>(*map_cloud_, min_cloud, max_cloud);
  min_cloud(0) -= 1;
  min_cloud(1) -= 1;
  max_cloud(0) += 1;
  max_cloud(1) += 1;
  float len_x = max_cloud(0) - min_cloud(0);
  float len_y = max_cloud(1) - min_cloud(1);

  resolution_ = std::max(len_x / (float)output_img_width_,
                         len_y / (float)output_img_height_);
  // std::cout << "grid resolution : " << resolution_ << std::endl;

  vis_origin_position_(0) =
      min_cloud(0) + 0.5 * resolution_ * output_img_width_;
  vis_origin_position_(1) =
      min_cloud(1) + 0.5 * resolution_ * output_img_height_;

  // 更新地图边界
  map_boundary_x_ = Eigen::Vector2d(min_cloud(0), max_cloud(0));
  map_boundary_y_ = Eigen::Vector2d(min_cloud(1), max_cloud(1));
  // 防止地图边沿
  refresh_map_x_ = Eigen::Vector2d(min_cloud(0) + 10 * resolution_,
                                   max_cloud(0) - 10 * resolution_);
  refresh_map_y_ = Eigen::Vector2d(min_cloud(1) + 10 * resolution_,
                                   max_cloud(1) - 10 * resolution_);

  CloudTypePtr map_cloud_ds(new CloudType);
  utils::DownsampleCloudAdapted(*map_cloud_, *map_cloud_ds, resolution_);

  vis_map_.create(output_img_height_, output_img_width_, CV_8UC1);
  vis_map_ = cv::Scalar(255);

  for (size_t i = 0; i < map_cloud_ds->size(); i++) {
    int index_y = int((map_cloud_ds->points[i].x - min_cloud(0)) / resolution_);
    int index_x = output_img_height_ -
                  int((map_cloud_ds->points[i].y - min_cloud(1)) / resolution_);
    if ((int)vis_map_.at<uchar>(index_x, index_y) >= 50) {
      vis_map_.at<uchar>(index_x, index_y) -= 20;
    }
  }

  cv::normalize(vis_map_, vis_map_, 0, 255, cv::NORM_MINMAX);

  visualization_flag_ = true;
  refresh_cloud_map_flag_ = false;
}

bool LidarMapper2DVisualizer::RefreshGlobalMap(
    const Eigen::Vector3d& _position) {
  if (map_cloud_->empty()) {
    AWARN_F("[LidarMapper2DVisualizer] map cloud is empty ");
    return false;
  }

  bool refresh_result = false;
  // 位姿态判断
  if (_position(0) < refresh_map_x_(0) || _position(0) > refresh_map_x_(1) ||
      _position(1) < refresh_map_y_(0) || _position(1) > refresh_map_y_(1)) {
    refresh_cloud_map_flag_ = true;
  }

  if (refresh_cloud_map_flag_) {
    vis_origin_position_ = _position;
    UpdateBoundaryParams();

    vis_map_.create(output_img_height_, output_img_width_, CV_8UC1);
    vis_map_ = cv::Scalar(255);
    // vis_map_ = cv::Mat(output_img_height_, output_img_width_, CV_8UC1,
    //                    cv::Scalar(255));

    CloudTypePtr map_cloud_ds(new CloudType);
    utils::DownsampleCloudAdapted(*map_cloud_, *map_cloud_ds, resolution_);
    for (size_t i = 0; i < map_cloud_ds->size(); i++) {
      if (map_cloud_ds->points[i].x < map_boundary_x_(0) ||
          map_cloud_ds->points[i].x > map_boundary_x_(1) ||
          map_cloud_ds->points[i].y < map_boundary_y_(0) ||
          map_cloud_ds->points[i].y > map_boundary_y_(1)) {
        continue;
      }

      int index_y =
          int((map_cloud_ds->points[i].x - map_boundary_x_(0)) / resolution_);
      int index_x =
          output_img_height_ -
          int((map_cloud_ds->points[i].y - map_boundary_y_(0)) / resolution_);
      if ((int)vis_map_.at<uchar>(index_x, index_y) >= 10) {
        vis_map_.at<uchar>(index_x, index_y) -= 10;
      }
    }

    cv::normalize(vis_map_, vis_map_, 0, 255, cv::NORM_MINMAX);

    refresh_cloud_map_flag_ = false;
    refresh_result = true;
  }

  visualization_flag_ = true;

  return refresh_result;
}

void LidarMapper2DVisualizer::UpdateBoundaryParams() {
  map_boundary_x_ = Eigen::Vector2d(
      vis_origin_position_(0) - 0.5 * output_img_width_ * resolution_,
      vis_origin_position_(0) + 0.5 * output_img_width_ * resolution_);
  map_boundary_y_ = Eigen::Vector2d(
      vis_origin_position_(1) - 0.5 * output_img_height_ * resolution_,
      vis_origin_position_(1) + 0.5 * output_img_height_ * resolution_);

  refresh_map_x_ = Eigen::Vector2d(
      vis_origin_position_(0) - 0.375 * output_img_width_ * resolution_,
      vis_origin_position_(0) + 0.375 * output_img_width_ * resolution_);
  refresh_map_y_ = Eigen::Vector2d(
      vis_origin_position_(1) - 0.375 * output_img_height_ * resolution_,
      vis_origin_position_(1) + 0.375 * output_img_height_ * resolution_);
}

bool LidarMapper2DVisualizer::GetVisulizationImage(cv::Mat& _image) {
  if (!visualization_flag_) {
    return false;
  }

  vis_map_.copyTo(_image);

  return true;
}

float LidarMapper2DVisualizer::GetResolution() {
  if (!visualization_flag_) {
    return 0;
  }
  int ten_resolution = int(resolution_ * 10);
  return ten_resolution * 0.1;
}

}  // namespace multi_sensor_mapping