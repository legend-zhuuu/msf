#ifndef MSM_LIDAR_MAPPER_2D_VISUALIZER_H
#define MSM_LIDAR_MAPPER_2D_VISUALIZER_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <thread>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief The LidarMapper2DVisualizer class 激光建图2D可视化
 *
 */
class LidarMapper2DVisualizer {
 public:
  typedef std::shared_ptr<LidarMapper2DVisualizer> Ptr;

  /**
   * @brief Construct a new Lidar Mapper 2D Visualizer object
   *
   */
  LidarMapper2DVisualizer();

  /**
   * @brief Set the Map Resolution object 设置地图分辨率
   *
   * @param _resolution
   */
  void SetMapResolution(float _resolution);

  /**
   * @brief 位置转换为像素坐标系
   *
   * @param _position
   * @param _pix_x
   * @param _pix_y
   * @return true
   * @return false
   */
  bool PoseToPixelCoordinate(const Eigen::Vector3d& _position, int& _pix_x,
                             int& _pix_y);

  /**
   * @brief 姿态转换为像素坐标
   *
   * @param _position
   * @param _rotation
   * @param _pix_x
   * @param _pix_y
   * @param _angle
   * @return true
   * @return false
   */
  bool PoseToPixelCoordinate(const Eigen::Vector3d& _position,
                             const Eigen::Quaterniond& _rotation, int& _pix_x,
                             int& _pix_y, float& _angle);

  /**
   * @brief Display cloud 显示点云
   *
   * @param _cloud
   */
  void DisplayCloud(const CloudTypePtr& _cloud, const Eigen::Vector3d& _postion,
                    const Eigen::Quaterniond& _rotation);

  /**
   * @brief Display global cloud 显示全局点云
   *
   * @param _cloud
   */
  void DisplayGlobalCloud(const CloudTypePtr& _cloud);

  /**
   * @brief 刷新全局地图
   *
   * @param _position
   */
  bool RefreshGlobalMap(const Eigen::Vector3d& _position);

  /**
   * @brief Get the Visulization Image object
   *
   * @param _image
   * @return true
   * @return false
   */
  bool GetVisulizationImage(cv::Mat& _image);

  /**
   * @brief Get the Resolution object 获取地图分辨率
   *
   * @return float
   */
  float GetResolution();

 private:
  /**
   * @brief update boundary params 更新边界参数
   *
   */
  void UpdateBoundaryParams();

 private:
  /// @brief 输出图像的宽
  int output_img_width_;
  /// @brief 输出图像的高
  int output_img_height_;
  /// @brief 分辨率
  float resolution_;
  /// @brief 可视化标志
  bool visualization_flag_;
  /// @brief 刷新点云地图标志位
  bool refresh_cloud_map_flag_;

  /// @brief 可视化原点位置
  Eigen::Vector3d vis_origin_position_;

  /// @brief 地图边界x
  Eigen::Vector2d map_boundary_x_;
  /// @brief 地图边界y
  Eigen::Vector2d map_boundary_y_;
  /// @brief 更新地图x边界
  Eigen::Vector2d refresh_map_x_;
  /// @brief 更新地图y边界
  Eigen::Vector2d refresh_map_y_;

  /// @brief 地图点云
  CloudTypePtr map_cloud_;

  /// @brief 可视化地图
  cv::Mat vis_map_;
};
}  // namespace multi_sensor_mapping

#endif