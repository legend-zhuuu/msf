#include "multi_sensor_mapping/spline/trajectory.h"

#include <inttypes.h>

#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

Trajectory::Trajectory(double time_interval, double start_time)
    : Se3Spline<SplineOrder, double>(time_interval * S_TO_NS,
                                     start_time * S_TO_NS),
      data_start_time_ns_(-1),
      active_time_ns_(-1),
      forced_fixed_time_ns_(-1),
      max_time_ns_(-1) {
  this->extendKnotsTo(start_time * S_TO_NS,
                      SO3d(Eigen::Quaterniond::Identity()),
                      Eigen::Vector3d(0, 0, 0));
}

void Trajectory::SetExtrinsicParams(
    const std::shared_ptr<SensorParams>& _sensor_params,
    const std::shared_ptr<ExtrinsicParams>& _extrinsic_params) {
  Eigen::Matrix4d mat_lidar_to_imu =
      _extrinsic_params->imu_to_baselink.inverse() *
      _extrinsic_params->lidar_to_baselink[_sensor_params->major_lidar_];
  ExtParam lidar_param;
  lidar_param.p = mat_lidar_to_imu.block<3, 1>(0, 3);
  lidar_param.q = Eigen::Quaterniond(mat_lidar_to_imu.block<3, 3>(0, 0));
  lidar_param.UpdateParam();
  sensor2imu_[LIDAR] = lidar_param;

  ExtParam imu_param;
  sensor2imu_[IMU] = imu_param;
}

int64_t Trajectory::GetDataStartTime() const { return data_start_time_ns_; }

int64_t Trajectory::GetActiveTime() const { return active_time_ns_; }

int64_t Trajectory::GetForcedFixedTime() const { return forced_fixed_time_ns_; }

double Trajectory::MinTime(const FrameType type) {
  double t_offset_ns = this->sensor2imu_[type].t_offset_ns;
  return NS_TO_S * this->minTimeNs() - NS_TO_S * t_offset_ns;
}

double Trajectory::MaxTime(const FrameType type) {
  double t_offset_ns = sensor2imu_[type].t_offset_ns;
  return NS_TO_S * this->maxTimeNs() - NS_TO_S * t_offset_ns;
}

int64_t Trajectory::MinTimeNs(const FrameType type) {
  double t_offset_ns = sensor2imu_[type].t_offset_ns;
  return this->minTimeNs() - t_offset_ns;
}

int64_t Trajectory::MaxTimeNs(const FrameType type) {
  double t_offset_ns = sensor2imu_[type].t_offset_ns;
  return this->maxTimeNs() - t_offset_ns;
}

void Trajectory::GetIMUState(int64_t time_ns, IMUState& imu_state) {
  SE3d pose = GetIMUPose(time_ns);

  imu_state.timestamp_ns = time_ns;
  imu_state.orientation = pose.unit_quaternion();
  imu_state.position = pose.translation();
  imu_state.velocity = GetTransVelWorld(time_ns);
  // imu_state.bias;
  // imu_state.g;
}

Eigen::Vector3d Trajectory::GetPositionWorld(const int64_t timestamp) {
  return this->positionWorld(timestamp);
}

Eigen::Vector3d Trajectory::GetTransVelWorld(const int64_t timestamp) {
  return this->transVelWorld(timestamp);
}

Eigen::Vector3d Trajectory::GetTransAccelWorld(const int64_t timestamp) {
  return this->transAccelWorld(timestamp);
}

Eigen::Vector3d Trajectory::GetRotVelBody(const int64_t timestamp) {
  return this->rotVelBody(timestamp);
}

SE3d Trajectory::GetIMUPose(const int64_t timestamp) {
  return this->poseNs(timestamp);
}

SE3d Trajectory::GetSensorPose(const int64_t timestamp, const FrameType type) {
  auto ep = sensor2imu_[type];
  int64_t time_ns = timestamp + ep.t_offset_ns;
  if (time_ns < this->minTimeNs() || time_ns >= this->maxTimeNs()) {
    AWARN_F("[Trajectory] GetSensorPose failed . Querry time [%" PRId64
            "]   not in range [ %" PRId64 " ~  %" PRId64 "]",
            time_ns, this->minTimeNs(), this->maxTimeNs());
  }

  SE3d pose_I_to_G = this->poseNs(time_ns);
  SE3d pose_S_to_G = pose_I_to_G * ep.se3;
  return pose_S_to_G;
}

void Trajectory::SetForcedFixedTime(double time) {
  if (time < MinTime(LIDAR))
    forced_fixed_time_ns_ = MinTime(LIDAR) * NS_TO_S;
  else
    forced_fixed_time_ns_ = time * NS_TO_S;
}

void Trajectory::SetActiveTime(int64_t time) { active_time_ns_ = time; }

void Trajectory::SetDataStartTime(int64_t time) { data_start_time_ns_ = time; }

std::map<FrameType, ExtParam> Trajectory::GetSensorExtParams() {
  return sensor2imu_;
}

ExtParam Trajectory::GetExtParam(const FrameType type) {
  return sensor2imu_[type];
}

}  // namespace multi_sensor_mapping
