#include "multi_sensor_mapping/frontend/inertial/ImuRotationPredictor.h"

namespace multi_sensor_mapping {

ImuRotationPredictor::ImuRotationPredictor(
    ExtrinsicParams::Ptr _extrinsic_params)
    : ImuPredictorBase(_extrinsic_params) {}

void ImuRotationPredictor::AddImuData(const IMUData &_imu_data) {
  if (!imu_data_cache_.empty() &&
      _imu_data.timestamp <= imu_data_cache_.back().timestamp) {
    return;
  }
  imu_data_cache_.push_back(_imu_data);
}

void ImuRotationPredictor::CorrectPose(double timestamp, const Transf &pose,
                                       bool is_degenerate, FrameType frame) {
  latest_pose_ = PoseToImuFrame(pose, frame);
  quater_at_latest_time_ = FindImuRotation(timestamp);
  predict_valid_flag_ = true;
}

bool ImuRotationPredictor::Predict(double timestamp, Transf &sensor_pose,
                                   FrameType frame) {
  if (!predict_valid_flag_) {
    LOG(WARNING) << "Please correct pose first";
    sensor_pose = Transf::Identity();
    return false;
  }

  Transf d_trans = Transf::Identity();
  d_trans.block<3, 3>(0, 0) =
      (quater_at_latest_time_.inverse() * FindImuRotation(timestamp))
          .toRotationMatrix();
  Transf predict_imu_pose = latest_pose_ * d_trans;
  sensor_pose = ImuPoseToSensorFrame(predict_imu_pose, frame);
  return true;
}

Transf ImuRotationPredictor::PoseToImuFrame(const Transf &pose,
                                            FrameType frame) {
  return pose * extrinsic_matrix_[frame].inverse();
}

Transf ImuRotationPredictor::ImuPoseToSensorFrame(const Transf &pose,
                                                  FrameType frame) {
  return pose * extrinsic_matrix_[frame];
}

Eigen::Quaternionf ImuRotationPredictor::FindImuRotation(const double time) {
  if (imu_data_cache_.empty()) {
    LOG(INFO) << "imu_data_ is empty";
    return Eigen::Quaternionf::Identity();
  }
  if (time > imu_data_cache_.back().timestamp)
    return imu_data_cache_.back().orientation.unit_quaternion().cast<float>();
  if (time < imu_data_cache_.front().timestamp)
    return imu_data_cache_.front().orientation.unit_quaternion().cast<float>();
  for (size_t i = 0; i < imu_data_cache_.size(); i++) {
    if (time < imu_data_cache_[i].timestamp) {
      Eigen::Quaterniond q_back =
          imu_data_cache_[i - 1].orientation.unit_quaternion();
      Eigen::Quaterniond q_front =
          imu_data_cache_[i].orientation.unit_quaternion();
      double q_ratio =
          (time - imu_data_cache_[i - 1].timestamp) /
          (imu_data_cache_[i].timestamp - imu_data_cache_[i - 1].timestamp);
      return q_back.slerp(q_ratio, q_front).cast<float>();
    }
  }
  return Eigen::Quaternionf::Identity();
}

}  // namespace multi_sensor_mapping
