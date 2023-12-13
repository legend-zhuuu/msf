#include <lqc/utils/imu.h>

namespace lqc {

SimpleAccelFilter::SimpleAccelFilter(int buffer_size) {
  orn_.setIdentity();
  accel_buf_.reserve(buffer_size);
  accel_.setZero();
}

void SimpleAccelFilter::reset() {
  accel_buf_.clear();
}

void SimpleAccelFilter::update(const sdk::LowState &state) {
  const auto &imu = state.imu;
  orn_.set(imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]);

  // convert to world frame
  Vec3 accel{imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]};
  accel_ << orn_.rotate(accel);
  accel_ += gravity_;
  accel_buf_.push_back(accel_);
}

const Eigen::Array3f &SimpleAccelFilter::getAccel() const {
  return accel_;
}

Eigen::Array3f SimpleAccelFilter::getFilteredAccel() const {
  if (accel_buf_.is_empty()) return Eigen::Array3f::Zero();
  Eigen::Array3f filtered_accel = Eigen::Array3f::Zero();
  for (const auto &accel : accel_buf_) { filtered_accel += accel; }
  filtered_accel /= accel_buf_.size();
  return orn_.inv().rotate(filtered_accel);
}

}  // namespace lqc

