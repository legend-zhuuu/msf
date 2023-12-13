#include "multi_sensor_mapping/frontend/imu/imu_initializer.h"

#include "multi_sensor_mapping/utils/utils_eigen.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

ImuInitializer::ImuInitializer(double _gravity_norm, double _window_length,
                               double _imu_excite_threshold)
    : window_length_(_window_length),
      imu_excite_threshold_(_imu_excite_threshold),
      use_imu_data2_flag_(false) {
  window_length_ns_ = window_length_ * 1e9;
  gravity_ << 0, 0, _gravity_norm;
  gyro_bias_ = Eigen::Vector3d(0, 0, 0);
  accel_bias_ = Eigen::Vector3d(0, 0, 0);
}

void ImuInitializer::FeedImuData(const IMUData& _imu_data) {
  std::lock_guard<std::mutex> lg(imu_buffer_mutex_);
  imu_data_list_.push_back(_imu_data);

  auto iter = imu_data_list_.begin();
  while (iter != imu_data_list_.end() &&
         iter->timestamp < _imu_data.timestamp - 2 * window_length_) {
    iter = imu_data_list_.erase(iter);
  }
}

void ImuInitializer::FeedImuData(const IMUData2& _imu_data) {
  std::lock_guard<std::mutex> lg(imu_buffer_mutex_);
  imu_data2_list_.push_back(_imu_data);
  use_imu_data2_flag_ = true;

  auto iter = imu_data2_list_.begin();
  while (iter != imu_data2_list_.end() &&
         iter->timestamp_ns < _imu_data.timestamp_ns - 2 * window_length_ns_) {
    iter = imu_data2_list_.erase(iter);
  }
}

double ImuInitializer::ComputeImuStationaryCoeff() {
  if (use_imu_data2_flag_) {
    if (imu_data2_list_.size() < 2) return -1;

    int64_t newest_time = imu_data2_list_.back().timestamp_ns;
    int64_t oldest_time = imu_data2_list_.front().timestamp_ns;

    if (newest_time - oldest_time < window_length_ns_) return -1;

    std::vector<IMUData2> check_window;

    {
      std::lock_guard<std::mutex> lg(imu_buffer_mutex_);
      for (const IMUData2& data : imu_data2_list_) {
        if (data.timestamp_ns > newest_time - 1 * window_length_ns_ &&
            data.timestamp_ns <= newest_time - 0 * window_length_ns_) {
          check_window.push_back(data);
        }
      }
    }
    if (check_window.empty()) {
      return -1;
    }

    Eigen::Vector3d a_avg = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < check_window.size(); i++) {
      a_avg += check_window.at(i).accel;
    }
    a_avg = a_avg / check_window.size();

    double a_var = 0;
    for (const IMUData2& data : check_window) {
      a_var += (data.accel - a_avg).dot(data.accel - a_avg);
    }
    a_var = std::sqrt(a_var / ((int)check_window.size() - 1));

    return a_var;
  }

  if (imu_data_list_.size() < 2) return -1;

  double newest_time = imu_data_list_.back().timestamp;
  double oldest_time = imu_data_list_.front().timestamp;

  if (newest_time - oldest_time < window_length_) return -1;

  std::vector<IMUData> check_window;

  {
    std::lock_guard<std::mutex> lg(imu_buffer_mutex_);
    for (const IMUData& data : imu_data_list_) {
      if (data.timestamp > newest_time - 1 * window_length_ &&
          data.timestamp <= newest_time - 0 * window_length_) {
        check_window.push_back(data);
      }
    }
  }
  if (check_window.empty()) {
    return -1;
  }

  Eigen::Vector3d a_avg = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < check_window.size(); i++) {
    a_avg += check_window.at(i).accel;
  }
  a_avg = a_avg / check_window.size();

  double a_var = 0;
  for (const IMUData& data : check_window) {
    a_var += (data.accel - a_avg).dot(data.accel - a_avg);
  }
  a_var = std::sqrt(a_var / ((int)check_window.size() - 1));

  return a_var;
}

bool ImuInitializer::CheckImuStaticState(double& _start_time) {
  if (imu_data_list_.size() < 2) return false;

  double newest_time = imu_data_list_.back().timestamp;
  double oldest_time = imu_data_list_.front().timestamp;

  if (newest_time - oldest_time < window_length_) return false;

  std::vector<IMUData> check_window;
  for (const IMUData& data : imu_data_list_) {
    if (data.timestamp > newest_time - 1 * window_length_ &&
        data.timestamp <= newest_time - 0 * window_length_) {
      check_window.push_back(data);
    }
  }
  if (check_window.empty()) {
    return false;
  }

  Eigen::Vector3d a_avg = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < check_window.size(); i++) {
    a_avg += check_window.at(i).accel;
    w_avg += check_window.at(i).gyro;
  }
  a_avg = a_avg / check_window.size();
  w_avg = w_avg / check_window.size();
  double a_var = 0;
  for (const IMUData& data : check_window) {
    a_var += (data.accel - a_avg).dot(data.accel - a_avg);
  }
  a_var = std::sqrt(a_var / ((int)check_window.size() - 1));

  if (a_var > imu_excite_threshold_) {
    AINFO_F(
        "[ImuInitializer] To Much IMU excitation,  %f ,  above thrshold %f ",
        a_var, imu_excite_threshold_);

    return false;
  }

  Eigen::Vector3d z_axis = a_avg / a_avg.norm();

  // Create an x_axis
  Eigen::Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get z from the cross product of these two
  Eigen::Matrix<double, 3, 1> y_axis = Eigen::SkewSymmetric(z_axis) * x_axis;

  // From these axes get rotation
  Eigen::Matrix<double, 3, 3> Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  double roll, pitch, yaw;
  tf::Matrix3x3(Ro(0, 0), Ro(0, 1), Ro(0, 2), Ro(1, 0), Ro(1, 1), Ro(1, 2),
                Ro(2, 0), Ro(2, 1), Ro(2, 2))
      .getRPY(roll, pitch, yaw);

  _start_time = (newest_time + oldest_time) / 2;

  AINFO_F("[ImuInitializer] Init IMU rot(RPY)) :  %f , %f , %f , time < %.6f >",
          roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180, _start_time);

  // Create our state variables
  rot_GtoI_ = Eigen::Quaterniond(Ro);

  return true;
}

bool ImuInitializer::CheckImuStaticState(IMUState& _imu_state) {
  if (imu_data2_list_.size() < 2) return false;

  int64_t newest_time = imu_data2_list_.back().timestamp_ns;
  int64_t oldest_time = imu_data2_list_.front().timestamp_ns;

  if (newest_time - oldest_time < window_length_ns_) return false;

  std::vector<IMUData2> check_window;
  for (const IMUData2& data : imu_data2_list_) {
    if (data.timestamp_ns > newest_time - 1 * window_length_ns_ &&
        data.timestamp_ns <= newest_time - 0 * window_length_ns_) {
      check_window.push_back(data);
    }
  }
  if (check_window.empty()) {
    return false;
  }

  Eigen::Vector3d a_avg = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < check_window.size(); i++) {
    a_avg += check_window.at(i).accel;
    w_avg += check_window.at(i).gyro;
  }
  a_avg = a_avg / check_window.size();
  w_avg = w_avg / check_window.size();
  double a_var = 0;
  for (const IMUData2& data : check_window) {
    a_var += (data.accel - a_avg).dot(data.accel - a_avg);
  }
  a_var = std::sqrt(a_var / ((int)check_window.size() - 1));

  if (a_var > imu_excite_threshold_) {
    AINFO_F(
        "[ImuInitializer] To Much IMU excitation,  %f ,  above thrshold %f ",
        a_var, imu_excite_threshold_);

    return false;
  }

  Eigen::Vector3d z_axis = a_avg / a_avg.norm();

  // Create an x_axis
  Eigen::Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get z from the cross product of these two
  Eigen::Matrix<double, 3, 1> y_axis = Eigen::SkewSymmetric(z_axis) * x_axis;

  // From these axes get rotation
  Eigen::Matrix<double, 3, 3> Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  double roll, pitch, yaw;
  tf::Matrix3x3(Ro(0, 0), Ro(0, 1), Ro(0, 2), Ro(1, 0), Ro(1, 1), Ro(1, 2),
                Ro(2, 0), Ro(2, 1), Ro(2, 2))
      .getRPY(roll, pitch, yaw);

  AINFO_F("[ImuInitializer] Init IMU rot(RPY)) :  %f , %f , %f ",
          roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180);

  // Create our state variables
  rot_GtoI_ = Eigen::Quaterniond(Ro);

  _imu_state.timestamp_ns = newest_time;
  _imu_state.orientation = rot_GtoI_.inverse();

  return true;
}

void ImuInitializer::Clear() {
  imu_data_list_.clear();
  imu_data2_list_.clear();
  gyro_bias_ = Eigen::Vector3d(0, 0, 0);
  accel_bias_ = Eigen::Vector3d(0, 0, 0);
}

}  // namespace multi_sensor_mapping
