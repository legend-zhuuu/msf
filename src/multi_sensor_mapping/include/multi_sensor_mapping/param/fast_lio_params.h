#ifndef MSM_FAST_LIO_PARAMS_H
#define MSM_FAST_LIO_PARAMS_H

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief FastLIOParams Fast LIO 参数
 *
 */
class FastLIOParams : public ParamBase {
 public:
  typedef std::shared_ptr<ParamBase> Ptr;

  /**
   * @brief Construct a new Fast L I O Params object
   *
   * @param _name
   */
  FastLIOParams(std::string _name = "fast_lio_params");

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
  ParamType Type() const override { return ParamType::PARAM_TYPE_FAST_LIO; }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 点云过滤数量
  int point_filter_num;
  /// @brief 特征点降采样分辨率
  double feature_ds_leaf_size;
  /// @brief 地图降采样分辨率
  double map_ds_leaf_size;
  /// @brief 最大迭代次数
  int max_iterations;
  /// @brief cube长度
  double cube_length;

  double delete_range;

  /// @brief 激光最近距离
  double lidar_min_distance;
  /// @brief 激光最远距离
  double lidar_max_distance;
  /// @brief 角速度协方差
  double gyro_cov;
  /// @brief 加速度协方差
  double accel_cov;
  /// @brief 角速度bias协方差
  double gyro_bias_cov;
  /// @brief 加速度bias协方差
  double accel_bias_cov;

  /// @brief 关键帧添加阈值
  double keyframe_adding_threshold;
};

}  // namespace multi_sensor_mapping

#endif