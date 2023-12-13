#ifndef MSM_LIDAR_DETECTION_PARAMS_H
#define MSM_LIDAR_DETECTION_PARAMS_H

#include <vector>

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 激光检测参数
 *
 */
class LidarDetectionParams : public ParamBase {
 public:
  /**
   * @brief Construct a new Lidar Detection Params object
   *
   * @param _name
   */
  LidarDetectionParams(std::string _name = "lidar_detection_params");

  /**
   * @brief Load 重写基类加载函数
   * @param _path_of_yaml
   */
  void Load(std::string _path_of_yaml) override;

  /**
   * @brief Print 重写基类打印函数
   */
  void Print() override;

  /**
   * @brief Type 重写类型返回函数
   */
  ParamType Type() const override {
    return ParamType::PARAM_TYPE_LIDAR_DETECTION;
  }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 是否启用检测器
  bool enable_detection;
  /// @brief 激光tag数量
  int lidar_tag_num;
  /// @brief 每个平面的点数
  int num_points_for_plane;
  /// @brief 强度阈值
  float intensity_threshold;
  /// @brief 深度阈值
  float depth_threshold;
  /// @brief tag的长宽
  float tag_size;
  /// @brief 体素大小
  float leaf_size;
  /// @brief 单层高度
  int layer_size;
};
}  // namespace multi_sensor_mapping

#endif