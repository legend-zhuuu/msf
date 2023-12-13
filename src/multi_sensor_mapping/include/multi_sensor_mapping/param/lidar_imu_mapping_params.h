#ifndef MSM_LIDAR_IMU_MAPPING_PARAMS_H
#define MSM_LIDAR_IMU_MAPPING_PARAMS_H

#include <vector>

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 激光-IMU参数
 *
 */
class LidarImuMappingParams : public ParamBase {
 public:
  typedef std::shared_ptr<LidarImuMappingParams> Ptr;

  /**
   * @brief Construct a new Sensor Params object
   *
   * @param _name
   */
  LidarImuMappingParams(std::string _name = "lidar_imu_mapping_params");

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
    return ParamType::PARAM_TYPE_LIDAR_IMU_MAPPING;
  }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 激光线数量
  int num_ring;
  /// @brief 激光扫描点数量
  int num_horizon;
  /// @brief 建图部分使用的线程数(非配准)
  int number_of_cores;

  /// @brief 角点阈值
  double edge_threshold;
  /// @brief 平面点阈值
  double surf_threshold;
  /// @brief 平面点降采样系数
  double surf_feature_leaf_size;

  /// @brief 激光最近距离
  double lidar_min_distance;
  /// @brief 激光最远距离
  double lidar_max_distance;

  /// @brief 配准方式
  MatchMethod match_method;

  /// @brief corner点云降采样参数
  double corner_ds_leaf_size;
  /// @brief surface点云降采样参数
  double surface_ds_leaf_size;
  /// @brief 当前帧提取的最少角点特征
  int corner_feature_min_valid_num;
  /// @brief 当前帧提取的最少平面特征
  int surface_feature_min_valid_num;

  /// @brief 关键帧搜索半径
  double keyframe_search_radius;
  /// @brief 添加关键帧距离阈值
  double keyframe_adding_distance_threshold;
  /// @brief 添加关键帧角度阈值
  double keyframe_adding_angle_threshold;
  /// @brief 关键帧降采样系数
  double keyframe_mapping_downsample_size;

  /// @brief 自动闭环检测
  bool auto_loop_closure;
  /// @brief 闭环搜索距离
  double loop_closure_searching_distance;
  /// @brief 闭环距离阈值
  double loop_closure_distance_interval;
  /// @brief 闭环分数阈值
  double loop_closure_score_threshold;
  /// @brief 单层运行
  bool mono_layer_motion;

  /// @brief 初始窗口长度
  double initial_window_length;
  /// @brief IMU激励阈值
  double imu_excite_threshold;

  /// @brief 重力系数
  double gravity_norm;
  /// @brief IMU参数
  double imu_accel_noise;
  double imu_gyro_noise;
  double imu_accel_bias_noise;
  double imu_gyro_bias_noise;

  /// @brief 地图分辨率
  double map_resolution;
};

}  // namespace multi_sensor_mapping

#endif
