#include "multi_sensor_mapping/frontend/lidar/gicp_scan_matcher.h"

namespace multi_sensor_mapping {

GICPScanMatcher::GICPScanMatcher() : resolution_(0.1), match_score_(-1) {
  cloud_filter_.setLeafSize(resolution_, resolution_, resolution_);
  InitMatcher();
}

GICPScanMatcher::GICPScanMatcher(double _resolution)
    : resolution_(_resolution), match_score_(-1) {
  cloud_filter_.setLeafSize(resolution_, resolution_, resolution_);
  InitMatcher();
}

void GICPScanMatcher::SetTargetCloud(const CloudTypePtr& _target_cloud) {
  CloudTypePtr target_cloud_ds_(new CloudType);
  DownSampleCloud(_target_cloud, target_cloud_ds_);
  gicp_ptr_->setInputTarget(target_cloud_ds_);
}

bool GICPScanMatcher::Match(const CloudTypePtr& _source_cloud,
                            const Eigen::Matrix4f& _initial_pose,
                            Eigen::Matrix4f& _pose_estimate) {
  match_score_ = -1;

  CloudTypePtr source_cloud_ds_(new CloudType);
  DownSampleCloud(_source_cloud, source_cloud_ds_);

  gicp_ptr_->setInputCloud(source_cloud_ds_);

  CloudType output;
  gicp_ptr_->align(output, _initial_pose);
  _pose_estimate = gicp_ptr_->getFinalTransformation();

  match_score_ = gicp_ptr_->getFitnessScore();
  return true;
}

double GICPScanMatcher::GetMatchScore() { return match_score_; }

void GICPScanMatcher::InitMatcher() {
  gicp_ptr_ = std::make_shared<
      pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>>();
}

void GICPScanMatcher::DownSampleCloud(CloudTypePtr _input_cloud,
                                      CloudTypePtr _output_cloud) {
  _output_cloud->clear();
  cloud_filter_.setInputCloud(_input_cloud);
  cloud_filter_.filter(*_output_cloud);
}

}  // namespace multi_sensor_mapping