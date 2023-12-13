#ifndef MSM_IMU_INITIALIZER_H
#define MSM_IMU_INITIALIZER_H

#include <memory>
#include <mutex>

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

/**
 * @brief The InertialInitializer class IMU静止初始化
 * Adapted from OpenVINS
 */
class ImuInitializer {
 public:
  typedef std::shared_ptr<ImuInitializer> Ptr;

  /**
   * @brief InertialInitializer 构造函数
   * @param _gravity_norm
   * @param _window_length
   * @param _imu_excite_threshold
   */
  ImuInitializer(double _gravity_norm, double _window_length,
                 double _imu_excite_threshold);

  /**
   * @brief FeedImuData 添加IMU数据
   * @param _imu_data
   */
  void FeedImuData(const IMUData& _imu_data);

  /**
   * @brief FeedImuData 添加IMU数据
   *
   * @param _imu_data
   */
  void FeedImuData(const IMUData2& _imu_data);

  /**
   * @brief 计算IMU的静止系数
   *
   * @return double
   */
  double ComputeImuStationaryCoeff();

  /**
   * @brief SystemStaticStateCheck 判断IMU的静止状态
   * @return
   */
  bool CheckImuStaticState(double& _start_time);

  /**
   * @brief
   *
   * @param _imu_state
   * @return true
   * @return false
   */
  bool CheckImuStaticState(IMUState& _imu_state);

  /**
   * @brief 清除状态
   *
   */
  void Clear();

  /**
   * @brief GetGravity 获取重力
   * @return
   */
  inline Eigen::Vector3d GetGravity() { return gravity_; }

  /**
   * @brief GetGyroBias 获取角速度bias
   * @return
   */
  inline Eigen::Vector3d GetGyroBias() { return gyro_bias_; }

  /**
   * @brief GetAccelBias 获取加速度bias
   * @return
   */
  inline Eigen::Vector3d GetAccelBias() { return accel_bias_; }

  /**
   * @brief GetLatestIMUTimestamp 获取最新的IMU时间戳
   * @return
   */
  inline double GetLatestIMUTimestamp() {
    return imu_data_list_.back().timestamp;
  }

  /**
   * @brief GetStartIMUTimestamp 获取IMU序列中最老的时间戳
   * @return
   */
  inline double GetStartIMUTimestamp() {
    return imu_data_list_.front().timestamp;
  }

  /**
   * @brief GetI0ToG 获取建图第一帧数据在世界坐标系的旋转
   * @return
   */
  inline Eigen::Quaterniond GetI0ToG() { return rot_GtoI_.inverse(); }

  /**
   * @brief GetLatestIMUData 获取最新的IMU数据
   *
   * @return IMUData2
   */
  inline IMUData2 GetLatestImuData() { return imu_data2_list_.back(); }

 private:
  /// @brief 滑窗时间
  double window_length_;
  /// @brief 滑窗时间
  int64_t window_length_ns_;
  /// @brief imu加速度激励阈值
  double imu_excite_threshold_;
  /// @brief 重力
  Eigen::Vector3d gravity_;
  /// @brief 陀螺仪bias
  Eigen::Vector3d gyro_bias_;
  /// @brief 加速度bias
  Eigen::Vector3d accel_bias_;
  /// @brief 缓存imu数据序列
  std::vector<IMUData> imu_data_list_;
  /// @brief 缓存imu数据序列
  std::vector<IMUData2> imu_data2_list_;
  /// @brief 世界坐标在IMU第一帧坐标系的旋转
  Eigen::Quaterniond rot_GtoI_;
  /// @brief IMU缓存锁
  std::mutex imu_buffer_mutex_;
  /// @brief 使用IMU2数据标志
  bool use_imu_data2_flag_;
};

}  // namespace multi_sensor_mapping

#endif
