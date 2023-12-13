#ifndef MSM_GRID_MAP_GENERATOR_H
#define MSM_GRID_MAP_GENERATOR_H

#include <opencv2/highgui/highgui_c.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief The GridMapGenerator class 栅格地图生成器
 */
class GridMapGenerator {
 public:
  /**
   * @brief GridMapGenerator
   * @param _cloud_path
   * @param _grid_resolution
   */
  GridMapGenerator(std::string _cloud_path, double _grid_resolution = 0.1);

  /**
   * @brief GridMapGenerator
   * @param _cloud
   * @param _grid_resolution
   */
  GridMapGenerator(CloudTypePtr _cloud, std::string _session_path,
                   double _grid_resolution = 0.1);

  /**
   * @brief Set the Key Frame Poses object
   *
   * @param _key_frame_poses
   */
  void SetKeyFramePoses(const std::vector<Eigen::Vector3d>& _key_frame_poses);

  /**
   * @brief Cloud2GreyImage
   */
  void Cloud2GreyImage();

  /**
   * @brief 基于点云强度生成地表特征地图
   */
  void GenerateGroundFeatureMap();

  /**
   * @brief DilateCallback
   * @param data
   */
  static void DilateCallback(int, void* data);

  /**
   * @brief GenerateMap
   */
  void GenerateMap();

  /**
   * @brief
   *
   * @param _map_name
   */
  void GenerateMap(std::string _map_name);

  /**
   * @brief SaveGridMap
   * @param _image
   */
  void SaveGridMap(cv::Mat& _image);

  /**
   * @brief 保存栅格地图
   *
   * @param _map_name
   * @param _image
   */
  void SaveGridMap(std::string _map_name, cv::Mat& _image);

 public:
  /// @brief 缓存路径
  std::string cache_path_;

  /// @brief 灰度图
  cv::Mat grey_image_;
  /// @brief 路线图
  cv::Mat road_map_;

  cv::Mat colorful_image_;

  /// @brief 栅格分辨率
  double grid_resolution_;
  /// @brief 全局点云
  CloudTypePtr global_cloud_;
  /// @brief 关键帧姿态序列
  std::vector<Eigen::Vector3d> key_frame_poses_;

  int dilate_size_ = -1;
  /// @brief 原点位置
  Eigen::Vector3d origin_position_;
};

}  // namespace multi_sensor_mapping

#endif
