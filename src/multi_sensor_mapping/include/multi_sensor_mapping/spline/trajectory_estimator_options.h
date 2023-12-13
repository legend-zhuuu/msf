#ifndef MSM_TRAEJCTORY_ESTIMATOR_OPTIONS_H
#define MSM_TRAEJCTORY_ESTIMATOR_OPTIONS_H

#include <stdint.h>

#include "multi_sensor_mapping/utils/utils_common.h"

namespace multi_sensor_mapping {

struct LockExtrinsic {
  bool lock_P = true;
  bool lock_R = true;
  bool lock_t_offset = true;
};

/**
 * @brief 轨迹优化参数
 *
 */
struct TrajectoryEstimatorOptions {
  TrajectoryEstimatorOptions() {
    LockExtrinsic le;
    lock_ext_params[IMU] = le;
    lock_ext_params[LIDAR] = le;
  }
  /// @brief 外参锁定参数
  std::map<FrameType, LockExtrinsic> lock_ext_params;

  /// @brief 样条参数先备份,优化完成后确认收敛再赋值
  bool spline_back_up = true;
  /// @brief 使用自动求导
  bool use_auto_diff = false;
  /// @brief 时间偏移量
  int64_t time_offset_padding_ns = 1e7;

  /// @brief 锁定轨迹
  bool lock_trajectory = false;
  /// @brief 锁定新增的平移控制点
  bool lock_translation;

  /// @brief 锁定加速度偏置
  bool lock_accel_bias = true;
  /// @brief 锁定角速度偏置
  bool lock_gyro_bias = true;
  /// @brief 锁定重力
  bool lock_gravity = true;

  // ======= Marginalization ======= //
  /// @brief 是否使用边缘化
  bool is_marg_state = false;

  int ctrl_to_be_opt_now = 0;
  int ctrl_to_be_opt_later = 0;

  bool marg_bias_param = true;

  bool marg_gravity_param = true;

  bool marg_t_offset_param = true;

  /// @brief 显示残差
  bool show_residual_summary = false;
};
}  // namespace multi_sensor_mapping

#endif