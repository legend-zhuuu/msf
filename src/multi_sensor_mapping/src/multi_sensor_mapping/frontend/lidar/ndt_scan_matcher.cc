#include "multi_sensor_mapping/frontend/lidar/ndt_scan_matcher.h"

namespace multi_sensor_mapping {

NdtScanMatcher::NdtScanMatcher()
    : target_resolution_(1.0),
      source_resolution_(0.5),
      num_cores_(2),
      match_score_(-1) {
  cloud_filter_.setLeafSize(source_resolution_, source_resolution_,
                            source_resolution_);
  InitMatcher();
}

NdtScanMatcher::NdtScanMatcher(double _target_resolution,
                               double _source_resolution, int _num_cores)
    : target_resolution_(_target_resolution),
      source_resolution_(_source_resolution),
      num_cores_(_num_cores),
      match_score_(-1) {
  cloud_filter_.setLeafSize(source_resolution_, source_resolution_,
                            source_resolution_);
  InitMatcher();
}

void NdtScanMatcher::SetTargetCloud(CloudTypePtr &_target_cloud) {
  ndt_ptr_->setInputTarget(_target_cloud);
}

bool NdtScanMatcher::Match(CloudTypePtr _source_cloud,
                           const Eigen::Matrix4f &_initial_pose,
                           Eigen::Matrix4f &_pose_estimate) {
  match_score_ = -1;

  CloudTypePtr source_cloud_ds_(new CloudType);
  DownSampleCloud(_source_cloud, source_cloud_ds_);
  ndt_ptr_->setInputCloud(source_cloud_ds_);
  CloudType output;
  ndt_ptr_->align(output, _initial_pose);
  _pose_estimate = ndt_ptr_->getFinalTransformation();

  match_score_ = ndt_ptr_->getFitnessScore();
  return true;
}

double NdtScanMatcher::GetMatchScore() { return match_score_; }

void NdtScanMatcher::InitMatcher() {
  ndt_ptr_ = std::make_shared<
      pclomp::NormalDistributionsTransform<PointType, PointType> >();
  ndt_ptr_->setNumThreads(num_cores_);
  ndt_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_ptr_->setTransformationEpsilon(0.01);
  ndt_ptr_->setStepSize(0.1);
  ndt_ptr_->setResolution(target_resolution_);
  ndt_ptr_->setMaximumIterations(30);
  ndt_ptr_->setOulierRatio(0.15);
  ndt_ptr_->setEuclideanFitnessEpsilon(1e-3);
}

void NdtScanMatcher::DownSampleCloud(CloudTypePtr _input_cloud,
                                     CloudTypePtr _output_cloud) {
  _output_cloud->clear();
  cloud_filter_.setInputCloud(_input_cloud);
  cloud_filter_.filter(*_output_cloud);
}

}  // namespace multi_sensor_mapping
