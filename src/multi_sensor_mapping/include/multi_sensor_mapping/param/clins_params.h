#ifndef MSM_CLINS_PARAMS_H
#define MSM_CLINS_PARAMS_H

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief CLINS参数
 *
 */
class ClinsParams : public ParamBase {
 public:
  typedef std::shared_ptr<ClinsParams> Ptr;

  /**
   * @brief Construct a new Clins Params object
   *
   * @param _name
   */
  ClinsParams(std::string _name = "clins_params");

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
  ParamType Type() const override { return ParamType::PARAM_TYPE_CLINS; }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 控制点距离
  double knot_distance;
  /// @brief  更新k个控制点
  int update_every_k_knot;
  /// @brief 点云过滤数量
  int point_filter_num;

  /// @brief 激光最近距离
  double lidar_min_distance;
  /// @brief 激光最远距离
  double lidar_max_distance;

  /// @brief 特征点云降采样参数
  double feature_leaf_size;

  /// @brief 添加关键帧的时间阈值
  double keyframe_adding_time_threshold;
  /// @brief 添加关键帧距离阈值
  double keyframe_adding_distance_threshold;
  /// @brief 添加关键帧角度阈值
  double keyframe_adding_angle_threshold;
  /// @brief 关键帧降采样系数
  double keyframe_mapping_downsample_size;

  /// @brief 激光权重
  double lidar_weight;
  /// @brief 全局速度权重
  double global_velocity_weight;

  /// @brief 初始窗口长度
  double initial_window_length;
  /// @brief IMU激励阈值
  double imu_excite_threshold;

  /// @brief 重力系数
  double gravity_norm;

  /// @brief IMU频率
  double imu_frequency;
  /// @brief IMU参数
  double imu_accel_noise;
  double imu_gyro_noise;
  double imu_accel_bias_noise;
  double imu_gyro_bias_noise;
};
}  // namespace multi_sensor_mapping

#endif