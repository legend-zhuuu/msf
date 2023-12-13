#ifndef MSM_OPTIMIZATION_WEIGHT_H
#define MSM_OPTIMIZATION_WEIGHT_H

#include <Eigen/Eigen>
#include <memory>

namespace multi_sensor_mapping {

class ClinsParams;

/**
 * @brief IMU噪声参数
 *
 */
class ImuNoise {
 public:
  /**
   * @brief Construct a new Imu Noise object 构造函数
   *
   */
  ImuNoise();

  /**
   * @brief Construct a new Imu Noise object 构造函数
   *
   * @param _clins_params
   */
  ImuNoise(const std::shared_ptr<ClinsParams>& _clins_params);

  /**
   * @brief 打印显示
   *
   */
  void Print();

 public:
  /// @brief IMU频率
  double imu_frequency_ = 200.;

  double imu_rate_gyro_ = 1.0;
  double imu_rate_accel_ = 1.0;

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w_ = 1.6968e-04;

  /// Gyroscope white noise covariance
  double sigma_w_2_ = std::pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb_ = 1.9393e-05;

  /// Gyroscope random walk covariance
  double sigma_wb_2_ = std::pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a_ = 2.0000e-3;

  /// Accelerometer white noise covariance
  double sigma_a_2_ = std::pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab_ = 3.0000e-03;

  /// Accelerometer random walk covariance
  double sigma_ab_2_ = std::pow(3.0000e-03, 2);
};

/**
 * @brief 优化权重
 *
 */
class OptimizationWeight {
 public:
  /**
   * @brief Construct a new Optimization Weight object
   *
   */
  OptimizationWeight();

  /**
   * @brief Construct a new Optimization Weight object
   *
   * @param _clins_params
   */
  OptimizationWeight(const std::shared_ptr<ClinsParams>& _clins_params);

 public:
  /// @brief IMU噪声
  ImuNoise imu_noise_;
  /// @brief IMU信息矩阵
  Eigen::Matrix<double, 6, 1> imu_info_vec_;
  /// @brief bias信息矩阵
  Eigen::Matrix<double, 6, 1> bias_info_vec_;
  /// @brief 雷达权重
  double lidar_weight_;
  /// @brief 全局速度权重
  double global_velocity_weight_;
};

}  // namespace multi_sensor_mapping

#endif