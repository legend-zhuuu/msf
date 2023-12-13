#include "multi_sensor_mapping/core/lidar_imu_mapping_core.h"

#include "multi_sensor_mapping/frontend/imu/imu_pose_predictor.h"
#include "multi_sensor_mapping/frontend/lidar/feature_extractor.h"
#include "multi_sensor_mapping/frontend/lidar/loam_scan_matcher.h"
#include "multi_sensor_mapping/frontend/lidar/scan_undistortion.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

LidarImuMappingCore::LidarImuMappingCore()
    : param_initialized_flag_(false),
      first_scan_flag_(true),
      keyframe_flag_(false) {}

LidarImuMappingCore::~LidarImuMappingCore() {}

void LidarImuMappingCore::SetParams(
    const std::shared_ptr<SensorParams> &_sensor_params,
    const std::shared_ptr<ExtrinsicParams> &_extrinsic_params,
    const std::shared_ptr<LidarImuMappingParams> &_mapper_params) {
  sensor_params_ = _sensor_params;
  extrinsic_params_ = _extrinsic_params;
  mapper_params_ = _mapper_params;
  param_initialized_flag_ = true;
}

void LidarImuMappingCore::SetLidarMapSession(
    const std::shared_ptr<LidarMapSession> &_map_session) {
  active_lidar_map_session_ptr_ = _map_session;
}

void LidarImuMappingCore::InitializeAll() {
  if (!param_initialized_flag_) {
    AWARN_F("[LidarImuMappingCore] No input params !");
  }

  // 初始化 ImuPosePredictor
  pose_predictor_ptr_ = std::make_shared<ImuPosePredictor>(
      extrinsic_params_, mapper_params_->gravity_norm,
      mapper_params_->imu_accel_noise, mapper_params_->imu_gyro_noise,
      mapper_params_->imu_accel_bias_noise,
      mapper_params_->imu_gyro_bias_noise);

  // 初始化 FeatureExtractor
  feature_extractor_ptr_ = std::make_shared<FeatureExtractor>(
      mapper_params_->num_ring, mapper_params_->num_horizon,
      mapper_params_->edge_threshold, mapper_params_->surf_threshold,
      mapper_params_->surf_feature_leaf_size);

  // 初始化 LOAMScanMatcher
  scan_matcher_ptr_ = std::make_shared<LOAMScanMatcher>(
      mapper_params_->corner_ds_leaf_size, mapper_params_->surface_ds_leaf_size,
      mapper_params_->corner_feature_min_valid_num,
      mapper_params_->surface_feature_min_valid_num,
      mapper_params_->number_of_cores, mapper_params_->match_method);

  // 初始化 ScanUndistortion
  scan_undistortion_ptr_ =
      std::make_shared<ScanUndistortion>(false, true, true);

  current_cloud_.reset(new CloudType());
  current_corner_cloud_raw_.reset(new CloudType());
  current_surface_cloud_raw_.reset(new CloudType());
  current_deskewed_cloud_raw_.reset(new VelRTPointCloud());
}

void LidarImuMappingCore::OdometryOneScan(
    TimedCloudData &cloud_data, const std::vector<IMUData> &imu_data_list) {
  if (cloud_data.timestamp < map_start_time_) {
    return;
  }

  keyframe_flag_ = false;
  current_cloud_timestamp_ = cloud_data.timestamp;

  CheckImuData(imu_data_list);

  /// Step1 点云去畸变
  DeskewScan(cloud_data, imu_data_list);

  /// Step2 特征提取
  FeatureExtraction();

  // 第一帧特殊处理
  if (first_scan_flag_) {
    first_scan_flag_ = false;

    InitPose(imu_data_list);

  } else {
    /// Step3 姿态预测
    Eigen::Matrix4d predict_pose;
    UpdatePrediction(imu_data_list, predict_pose);

    /// Step4 拼接滑窗地图
    BuildSlidingWindowMap();

    /// Step5 配准
    if (!MatchScan(predict_pose)) {
      AWARN_F("[LidarImuMappingCore] Match Scan Wrong < %f > ! Skip Scan!",
              current_cloud_timestamp_);
    }
  }

  /// Step6 更新预积分
  UpdatePredictor();

  /// Step7 更新关键帧
  KeyframeSelection();
}

void LidarImuMappingCore::SetInitializationStatus(double start_time,
                                                  Eigen::Quaterniond init_rot) {
  map_start_time_ = start_time;
  map_start_rot_ = init_rot;
}

void LidarImuMappingCore::OnlyScanUndistortion(
    TimedCloudData &cloud_data, const std::vector<IMUData> &_imu_data_list) {
  VelRTPointCloudPtr cloud_out(new VelRTPointCloud);
  for (size_t i = 0; i < _imu_data_list.size(); i++) {
    scan_undistortion_ptr_->InputImuData(_imu_data_list[i]);
  }
  scan_undistortion_ptr_->UndistortScan(cloud_data.timestamp, cloud_data.cloud,
                                        cloud_out);
}

PoseData LidarImuMappingCore::GetCurrentPose() {
  PoseData pose_data;
  pose_data.timestamp = current_cloud_timestamp_;
  pose_data.position = current_pose_.block<3, 1>(0, 3);
  pose_data.orientation = Eigen::Quaterniond(current_pose_.block<3, 3>(0, 0));
  return pose_data;
}

CloudTypePtr LidarImuMappingCore::GetCurrentCloud() { return current_cloud_; }

bool LidarImuMappingCore::IsKeyFrame() { return keyframe_flag_; }

void LidarImuMappingCore::InitPose(const std::vector<IMUData> &imu_data_list) {
  for (size_t i = 0; i < imu_data_list.size(); i++) {
    pose_predictor_ptr_->AddImuData(imu_data_list[i]);
  }

  current_pose_ = Eigen::Matrix4d::Identity();
  current_pose_.block<3, 3>(0, 0) = map_start_rot_.toRotationMatrix();

  double roll, pitch, yaw;
  tf::Matrix3x3(current_pose_(0, 0), current_pose_(0, 1), current_pose_(0, 2),
                current_pose_(1, 0), current_pose_(1, 1), current_pose_(1, 2),
                current_pose_(2, 0), current_pose_(2, 1), current_pose_(2, 2))
      .getRPY(roll, pitch, yaw);

  AINFO_F("[LidarImuMappingCore] First Scan rot(RPY)) : %f, %f, %f ",
          roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180);
}

void LidarImuMappingCore::DeskewScan(
    TimedCloudData &_cloud_in, const std::vector<IMUData> &_imu_data_list) {
  for (size_t i = 0; i < _imu_data_list.size(); i++) {
    scan_undistortion_ptr_->InputImuData(_imu_data_list[i]);
  }
  scan_undistortion_ptr_->UndistortScan(_cloud_in.timestamp, _cloud_in.cloud,
                                        current_deskewed_cloud_raw_);

  pcl::copyPointCloud(*current_deskewed_cloud_raw_, *current_cloud_);
}

void LidarImuMappingCore::FeatureExtraction() {
  // 角点特征提取
  feature_extractor_ptr_->ExtractLOAMFeature(current_deskewed_cloud_raw_,
                                             current_corner_cloud_raw_,
                                             current_surface_cloud_raw_);
  // AINFO_F("[LidarImuMappingCore] Corner feature : %i, surface feature : %i",
  //         (int)current_corner_cloud_raw_->size(),
  //         (int)current_surface_cloud_raw_->size());
}

void LidarImuMappingCore::UpdatePrediction(
    const std::vector<IMUData> &_imu_data_list,
    Eigen::Matrix4d &_out_prediction) {
  for (size_t i = 0; i < _imu_data_list.size(); i++) {
    pose_predictor_ptr_->AddImuData(_imu_data_list[i]);
  }

  Eigen::Matrix4d prediction_pose = Eigen::Matrix4d::Identity();
  if (!pose_predictor_ptr_->Predict(current_cloud_timestamp_,
                                    prediction_pose)) {
    AWARN_F("[LidarImuMappingCore] Predict pose Failed");
  }
  _out_prediction = prediction_pose;
}

void LidarImuMappingCore::BuildSlidingWindowMap() {
  if (keyframe_update_flag_ == false) return;

  /// TODO iKD-tree优化
  CloudTypePtr surrounding_current_corner_cloud_raw(new CloudType);
  CloudTypePtr surrounding_current_surface_cloud_raw(new CloudType);
  active_lidar_map_session_ptr_->ExtractSurroundingKeyFrames(
      current_cloud_timestamp_, mapper_params_->keyframe_search_radius,
      surrounding_current_corner_cloud_raw,
      surrounding_current_surface_cloud_raw);
  scan_matcher_ptr_->SetTargetFeature(surrounding_current_corner_cloud_raw,
                                      surrounding_current_surface_cloud_raw);
}

bool LidarImuMappingCore::MatchScan(const Eigen::Matrix4d &_prediction) {
  Eigen::Matrix4d predict = _prediction;
  Eigen::Matrix4d imu_pre = _prediction;
  predict(0, 3) = current_pose_(0, 3);
  predict(1, 3) = current_pose_(1, 3);
  predict(2, 3) = current_pose_(2, 3);

  scan_matcher_ptr_->SetSourceFeature(current_corner_cloud_raw_,
                                      current_surface_cloud_raw_);

  scan_matcher_ptr_->Align(predict, current_pose_);

  return CheckAlignResult(predict, current_pose_);
}

void LidarImuMappingCore::UpdatePredictor() {
  pose_predictor_ptr_->CorrectPose(current_cloud_timestamp_, current_pose_);
}

void LidarImuMappingCore::KeyframeSelection() {
  keyframe_update_flag_ = false;

  if (active_lidar_map_session_ptr_->GetMapUnitSize() == 0 ||
      CheckKeyScan(current_pose_)) {
    keyframe_update_flag_ = true;
    keyframe_flag_ = true;
  }

  if (keyframe_update_flag_ == true) {
    // 生成map_unit并保存
    // std::shared_ptr<LidarMapUnit> map_unit(new LidarMapUnit(
    //     current_cloud_timestamp_, current_cloud_, current_corner_cloud_raw_,
    //     current_surface_cloud_raw_, current_pose_));
    CloudTypePtr current_cloud_ds_(new CloudType);
    utils::UniformSampleCloud(*current_cloud_, *current_cloud_ds_, 0.1);
    std::shared_ptr<LidarMapUnit> map_unit(new LidarMapUnit(
        current_cloud_timestamp_, current_cloud_ds_, current_corner_cloud_raw_,
        current_surface_cloud_raw_, current_pose_));
    map_unit->anchor_point_id_ = current_anchor_id_;
    active_lidar_map_session_ptr_->AddMapUnit(map_unit);

    latest_key_frame_pose_inv_ = current_pose_.inverse();

    // AINFO_F("[LidarImuMappingCore] --Add Keyframe No.%i",
    //         active_lidar_map_session_ptr_->GetMapUnitSize() - 1);
  }
}

bool LidarImuMappingCore::CheckAlignResult(const Eigen::Matrix4d &_prediction,
                                           const Eigen::Matrix4d &_aligned) {
  Eigen::Matrix4d delta_pose = _prediction.inverse() * _aligned;
  double x, y, z, roll, pitch, yaw;
  utils::GetTranslationAndEulerAngles(delta_pose, x, y, z, roll, pitch, yaw);

  double dis_diff = sqrt(x * x + y * y + z * z);
  if (dis_diff > 0.4 || roll > 0.2 || pitch > 0.2 || yaw > 0.2) {
    AWARN_F(
        "[LidarImuMappingCore] Dis diff  < %f > , Roll diff < %f >, Pitch diff "
        "< %f >, Yaw diff < %f >",
        dis_diff, roll, pitch, yaw);

    return false;
  }
  return true;
}

bool LidarImuMappingCore::CheckKeyScan(const Eigen::Matrix4d &_current_pose) {
  // 若有关联锚点 添加关键帧
  // if (current_anchor_id_ >= 0) return true;
  Eigen::Matrix4d delta_pose = latest_key_frame_pose_inv_ * _current_pose;
  double x, y, z, roll, pitch, yaw;
  utils::GetTranslationAndEulerAngles(delta_pose, x, y, z, roll, pitch, yaw);

  if (sqrt(x * x + y * y + z * z) >
          mapper_params_->keyframe_adding_distance_threshold ||
      roll > mapper_params_->keyframe_adding_angle_threshold ||
      pitch > mapper_params_->keyframe_adding_angle_threshold ||
      yaw > mapper_params_->keyframe_adding_angle_threshold) {
    return true;
  }
  return false;
}

void LidarImuMappingCore::CheckImuData(
    const std::vector<IMUData> &imu_data_list) {
  if (imu_data_list.empty()) return;

  if (imu_data_list.front().gyro.norm() == 0 &&
      imu_data_list.front().accel.norm() == 0) {
    AWARN_F("[LidarImuMappingCore] IMU gyro accel data error !");
  }
}

}  // namespace multi_sensor_mapping
