#include "multi_sensor_mapping/utils/optimization_weight.h"

#include "multi_sensor_mapping/param/clins_params.h"

namespace multi_sensor_mapping {
ImuNoise::ImuNoise() {}

ImuNoise::ImuNoise(const std::shared_ptr<ClinsParams>& _clins_params) {
  imu_frequency_ = _clins_params->imu_frequency;
  imu_rate_gyro_ = 1.0;
  imu_rate_accel_ = 1.0;
  sigma_w_ = _clins_params->imu_gyro_noise;
  sigma_w_2_ = std::pow(_clins_params->imu_gyro_noise, 2);
  sigma_wb_ = _clins_params->imu_gyro_bias_noise;
  sigma_wb_2_ = std::pow(_clins_params->imu_gyro_bias_noise, 2);
  sigma_a_ = _clins_params->imu_accel_noise;
  sigma_a_2_ = std::pow(_clins_params->imu_accel_noise, 2);
  sigma_ab_ = _clins_params->imu_accel_bias_noise;
  sigma_ab_2_ = std::pow(_clins_params->imu_accel_bias_noise, 2);
}

void ImuNoise::Print() {
  printf("IMU Noise:\n");
  printf("\t- gyroscope_noise_density: %.6f\n", sigma_w_);
  printf("\t- accelerometer_noise_density: %.5f\n", sigma_a_);
  printf("\t- gyroscope_random_walk: %.7f\n", sigma_wb_);
  printf("\t- accelerometer_random_walk: %.6f\n", sigma_ab_);
  printf("\t- imu_info_vec_rate_gyro: %.3f\n", imu_rate_gyro_);
  printf("\t- imu_info_vec_rate_accel: %.3f\n", imu_rate_accel_);
}

OptimizationWeight::OptimizationWeight() {}

OptimizationWeight::OptimizationWeight(
    const std::shared_ptr<ClinsParams>& _clins_params) {
  imu_noise_ = ImuNoise(_clins_params);
  imu_noise_.Print();

  double sqrt_dt = std::sqrt(1.0 / imu_noise_.imu_frequency_);
  Eigen::Vector3d one3d = Eigen::Vector3d::Ones();

  Eigen::Matrix<double, 12, 1> Q_sqrt_inv;
  Q_sqrt_inv.block<3, 1>(0, 0) =
      1.0 / (imu_noise_.sigma_w_ / sqrt_dt) * one3d * imu_noise_.imu_rate_gyro_;
  Q_sqrt_inv.block<3, 1>(3, 0) = 1.0 / (imu_noise_.sigma_a_ / sqrt_dt) * one3d *
                                 imu_noise_.imu_rate_accel_;

  Q_sqrt_inv.block<3, 1>(6, 0) =
      1.0 / (imu_noise_.sigma_wb_ /* *sqrt_dt */) * one3d;
  Q_sqrt_inv.block<3, 1>(9, 0) =
      1.0 / (imu_noise_.sigma_ab_ /* *sqrt_dt */) * one3d;
  imu_info_vec_ = Q_sqrt_inv.block<6, 1>(0, 0);
  bias_info_vec_ = Q_sqrt_inv.block<6, 1>(6, 0);

  lidar_weight_ = _clins_params->lidar_weight;
  global_velocity_weight_ = _clins_params->global_velocity_weight;
}

}  // namespace multi_sensor_mapping