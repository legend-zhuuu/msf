#ifndef MSM_TRAJECTORY_H
#define MSM_TRAJECTORY_H

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_common.h"
#include "se3_spline.h"

namespace multi_sensor_mapping {

class ExtrinsicParams;
class SensorParams;

/**
 * @brief SE3轨迹，建立在IMU坐标系上
 *
 */
class Trajectory : public Se3Spline<SplineOrder, double> {
 public:
  static constexpr double NS_TO_S = 1e-9;  ///< Nanosecond to second conversion
  static constexpr double S_TO_NS = 1e9;   ///< Second to nanosecond conversion

  /**
   * @brief Construct a new Trajectory object 构造函数
   *
   * @param time_interval
   * @param start_time
   */
  Trajectory(double time_interval, double start_time = 0);

  /**
   * @brief Set the Extrinsic Params object 设置外参
   *
   * @param _extrinsic_params
   */
  void SetExtrinsicParams(
      const std::shared_ptr<SensorParams>& _sensor_params,
      const std::shared_ptr<ExtrinsicParams>& _extrinsic_params);

  /**
   * @brief Get the Data Start Time object 获取数据起始时间
   *
   * @return int64_t
   */
  int64_t GetDataStartTime() const;

  /**
   * @brief Get the Active Time object
   *
   * @return double
   */
  int64_t GetActiveTime() const;

  /**
   * @brief Get the Forced Fixed Time object
   *
   * @return double
   */
  int64_t GetForcedFixedTime() const;

  /**
   * @brief 轨迹最小时间(s)
   *
   * @param type
   * @return double
   */
  double MinTime(const FrameType type = FrameType::IMU);

  /**
   * @brief 轨迹最大时间(s)
   *
   * @param type
   * @return double
   */
  double MaxTime(const FrameType type = FrameType::IMU);

  /**
   * @brief 轨迹最小时间(ns)
   *
   * @param type
   * @return int64_t
   */
  int64_t MinTimeNs(const FrameType type = FrameType::IMU);

  /**
   * @brief 轨迹最大时间(ns)
   *
   * @param type
   * @return int64_t
   */
  int64_t MaxTimeNs(const FrameType type = FrameType::IMU);

  /**
   * @brief 获取IMU状态
   *
   * @param time_ns
   * @param imu_state
   */
  void GetIMUState(int64_t time_ns, IMUState& imu_state);

  /**
   * @brief Get the Position World object 获取位置
   *
   * @param timestamp
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d GetPositionWorld(const int64_t timestamp);

  /**
   * @brief Get the Trans Vel World object 获取速度
   *
   * @param timestamp
   * @param type
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d GetTransVelWorld(const int64_t timestamp);

  /**
   * @brief Get the Trans Accel World object 获取加速度
   *
   * @param timestamp
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d GetTransAccelWorld(const int64_t timestamp);

  /**
   * @brief Get the Rot Vel Body object 获取角速度
   *
   * @param timestamp
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d GetRotVelBody(const int64_t timestamp);

  /**
   * @brief 获取IMU位置
   *
   * @param timestamp
   * @return SE3d
   */
  SE3d GetIMUPose(const int64_t timestamp);

  /**
   * @brief Get the Sensor Pose object 获取传感器姿态
   *
   * @param timestamp
   * @param type
   * @return SE3d
   */
  SE3d GetSensorPose(const int64_t timestamp, const FrameType type);

  /**
   * @brief Set the Forced Fixed Time object 设置强制固定时间
   *
   * @param time
   */
  void SetForcedFixedTime(double time);

  /**
   * @brief Set the Active Time object 设置active time
   *
   * @param time
   */
  void SetActiveTime(int64_t time);

  /**
   * @brief Set the Data Start Time object 设置数据起始时间
   *
   * @param time
   */
  void SetDataStartTime(int64_t time);

  /**
   * @brief Get the Sensor Ext Params object 获取传感器外参
   *
   * @return std::map<FrameType, ExtParam>
   */
  std::map<FrameType, ExtParam> GetSensorExtParams();

  /**
   * @brief Get the Ext Param object 获取传感器外参
   *
   * @param type
   * @return ExtParam
   */
  ExtParam GetExtParam(const FrameType type);

  double opt_min_init_time_tmp = 0;
  int opt_init_fixed_idx_tmp = -1;

  double opt_min_lio_time_tmp = 0;

  double opt_min_loop_time_tmp = 0;
  int opt_loop_fixed_idx_tmp = -1;

 private:
  /// @brief 有效数据起始时间
  int64_t data_start_time_ns_;
  /// @brief 可查询轨迹的时间
  int64_t active_time_ns_;
  /// @brief 强制固定时间
  int64_t forced_fixed_time_ns_;
  /// @brief 轨迹最大时间
  int64_t max_time_ns_;

  /// @brief 传感器-IMU的外参
  std::map<FrameType, ExtParam> sensor2imu_;
};

}  // namespace multi_sensor_mapping

#endif