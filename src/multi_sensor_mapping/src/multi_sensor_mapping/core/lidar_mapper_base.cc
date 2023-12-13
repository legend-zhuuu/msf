#include "multi_sensor_mapping/core/lidar_mapper_base.h"

#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"

namespace multi_sensor_mapping {

LidarMapperBase::LidarMapperBase()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      bag_start_time_(0),
      bag_durr_(-1) {}

void LidarMapperBase::SetParams(const std::shared_ptr<ParamSet>& _param_set) {
  sensor_params_ = _param_set->GetSensorParams();
  extrinsic_params_ = _param_set->GetExtrinsicParams();
}

void LidarMapperBase::SetBagPath(std::string _bag_path) {
  bag_path_ = _bag_path;
}

void LidarMapperBase::SetMappingPeriod(double _start_time, double _map_durr) {
  bag_start_time_ = _start_time;
  bag_durr_ = _map_durr;
}

void LidarMapperBase::Init() {}

void LidarMapperBase::Start() {}

void LidarMapperBase::Stop() {}

void LidarMapperBase::RegExceptionCallback(
    const std::function<void(const MSMError&)>& _cb_excep) {
  cb_exception_ = _cb_excep;
}

void LidarMapperBase::RegLidarPoseCallback(
    const std::function<void(const PoseData&)>& _cb_lidar_pose) {
  cb_put_lidar_pose_ = _cb_lidar_pose;
}

void LidarMapperBase::RegKeyFrameCallback(
    const std::function<void(const CloudTypePtr&, const PoseData&)>&
        _cb_key_frame) {
  cb_put_key_frame_ = _cb_key_frame;
}

void LidarMapperBase::RegFrameCallback(
    const std::function<void(const CloudTypePtr&, const PoseData&)>&
        _cb_frame) {
  cb_put_frame_ = _cb_frame;
}

void LidarMapperBase::RegPutMappingProgress(
    const std::function<void(const double&)>& _cb_progress) {
  cb_put_mapping_progress_ = _cb_progress;
}

void LidarMapperBase::RegPutMapSession(
    const std::function<void(const std::shared_ptr<LidarMapSession>&)>&
        _cb_session) {
  cb_put_session_ = _cb_session;
}

void LidarMapperBase::RunException(const MSMError& _error) {
  if (cb_exception_) {
    cb_exception_(_error);
  }
}

void LidarMapperBase::RunPutLidarPose(const PoseData& _lidar_pose) {
  if (cb_put_lidar_pose_) {
    cb_put_lidar_pose_(_lidar_pose);
  }
}

void LidarMapperBase::RunPutKeyFrame(const CloudTypePtr& _cloud,
                                     const PoseData& _pose) {
  if (cb_put_key_frame_) {
    cb_put_key_frame_(_cloud, _pose);
  }
}

void LidarMapperBase::RunPutFrame(const CloudTypePtr& _cloud,
                                  const PoseData& _pose) {
  if (cb_put_frame_) {
    cb_put_frame_(_cloud, _pose);
  }
}

void LidarMapperBase::RunPutMappingProgress(const double& _progress) {
  if (cb_put_mapping_progress_) {
    cb_put_mapping_progress_(_progress);
  }
}

bool LidarMapperBase::IsRunning() { return start_flag_; }

}  // namespace multi_sensor_mapping