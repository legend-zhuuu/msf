#include "multi_sensor_mapping/frontend/imu/imu_state_estimator.h"

#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

ImuStateEstimator::ImuStateEstimator() {}

void ImuStateEstimator::FeedImuData(const IMUData2& _imu_data) {
  imu_data_cache_.push_back(_imu_data);
}

void ImuStateEstimator::Propagate(const IMUState& _imu_state,
                                  int64_t _from_timestamp,
                                  int64_t _to_timestamp) {
  if (imu_data_cache_.empty()) {
    AWARN_F("[ImuStateEstimator] IMU data cache is empty !");
  }
  if (imu_data_cache_.front().timestamp_ns > _to_timestamp) {
    AWARN_F("[ImuStateEstimator] Input time is too small. < %.3f, %.3f > !",
            _to_timestamp * 1e-9, imu_data_cache_.front().timestamp_ns * 1e-9);
  }

  // 获取时间间隔内的IMU数据
  Eigen::aligned_vector<IMUData2> imu_vec;
  auto iter = imu_data_cache_.begin();
  while (iter != imu_data_cache_.end()) {
    if (next(iter)->timestamp_ns < _from_timestamp) {
      iter = imu_data_cache_.erase(iter);
    } else if (iter->timestamp_ns < _to_timestamp) {
      imu_data_cache_.push_back(*iter);
      iter++;
    } else
      break;
  }

  if (imu_vec.size() < 1) {
    AINFO_F("[ImuStateEstimator] IMU cache size : %i ", (int)imu_vec.size());
    return;
  }

  // 修正临界IMU数据
  if (imu_vec[0].timestamp_ns < _from_timestamp && imu_vec.size() > 1) {
    IMUData2 mid_imudata;
    mid_imudata.timestamp_ns = _from_timestamp;
    int64_t dt_1 = mid_imudata.timestamp_ns - imu_vec[0].timestamp_ns;
    int64_t dt_2 = imu_vec[1].timestamp_ns - mid_imudata.timestamp_ns;

    double w2 = dt_1 / double(dt_1 + dt_2);
    double w1 = dt_2 / double(dt_1 + dt_2);
    if (dt_2 != 0) {
      mid_imudata.accel = w1 * imu_vec[0].accel + w2 * imu_vec[1].accel;
      mid_imudata.gyro = w1 * imu_vec[0].gyro + w2 * imu_vec[1].gyro;
      imu_vec[0] = mid_imudata;
    }
  }

  // 中值积分
  IMUState x_now = _imu_state;
  IMUBias bias = x_now.bias;  // bias
  for (size_t i = 1; i < imu_vec.size(); i++) {
    IMUData2 last_imu = imu_vec[i - 1];

    double dt = (imu_vec[i].timestamp_ns - last_imu.timestamp_ns) * 1e-9;
    x_now.timestamp_ns = imu_vec[i].timestamp_ns;
    Eigen::Vector3d un_acc_0 =
        x_now.orientation * (last_imu.accel - bias.accel_bias) - x_now.gravity;
    Eigen::Vector3d un_gyro =
        0.5 * (last_imu.gyro + imu_vec[i].gyro) - bias.gyro_bias;
    x_now.orientation *=
        Eigen::Quaterniond(1, un_gyro(0) * dt / 2, un_gyro(1) * dt / 2,
                           un_gyro(2) * dt / 2)
            .normalized();
    Eigen::Vector3d un_acc_1 =
        x_now.orientation * (imu_vec[i].accel - bias.accel_bias) -
        x_now.gravity;
    Eigen::Vector3d un_accel = 0.5 * (un_acc_0 + un_acc_1);
    x_now.position += x_now.velocity * dt + 0.5 * un_accel * dt * dt;
    x_now.velocity += un_accel * dt;
  }

  latest_state_ = x_now;
  latest_state_.timestamp_ns = imu_vec.back().timestamp_ns;

  propagate_start_state_ = _imu_state;
  propagate_start_state_.timestamp_ns = imu_vec.front().timestamp_ns;
}

}  // namespace multi_sensor_mapping