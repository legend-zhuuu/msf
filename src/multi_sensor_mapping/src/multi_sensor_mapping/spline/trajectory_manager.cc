#include "multi_sensor_mapping/spline/trajectory_manager.h"

#include "multi_sensor_mapping/frontend/imu/imu_state_estimator.h"
#include "multi_sensor_mapping/param/clins_params.h"
#include "multi_sensor_mapping/spline/trajectory.h"
#include "multi_sensor_mapping/spline/trajectory_estimator.h"
#include "multi_sensor_mapping/spline/trajectory_estimator_options.h"
#include "multi_sensor_mapping/utils/optimization_weight.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

TrajectoryManager::TrajectoryManager(
    const std::shared_ptr<ClinsParams>& _clins_params,
    const std::shared_ptr<Trajectory>& _trajectory)
    : trajectory_(_trajectory) {
  imu_state_estimator_ = std::make_shared<ImuStateEstimator>();
  optimization_weight_ = std::make_shared<OptimizationWeight>(_clins_params);
  lidar_prior_ctrl_id_ = std::make_pair(0, 0);
}

void TrajectoryManager::FeedImuData(const IMUData2& _imu_data) {
  if (trajectory_->GetDataStartTime() < 0) {
    trajectory_->SetDataStartTime(_imu_data.timestamp_ns);
  }
  imu_data_cache_.push_back(_imu_data);
  imu_data_cache_.back().timestamp_ns -= trajectory_->GetDataStartTime();
  imu_data_cache_.push_back(_imu_data);
}

void TrajectoryManager::PropagateTrajectory(int64_t _scan_time_min,
                                            int64_t _scan_time_max) {
  if (imu_data_cache_.size() < 2) {
    AWARN_F("[TrajectoryManager] IMU data is empty !");
    return;
  }

  // 1. 扩展轨迹
  ExtendTrajectory(_scan_time_max);

  // 2. 查询积分初始状态
  IMUState imu_state;
  int64_t integrate_start_time = 0;
  static bool first_integrate_flag = true;
  if (first_integrate_flag) {
    first_integrate_flag = false;
    imu_state.timestamp_ns = integrate_start_time;
    imu_state.orientation = original_pose_.orientation.unit_quaternion();
    imu_state.position = original_pose_.position;
    imu_state.velocity = Eigen::Vector3d(0, 0, 0);
  } else {
    int64_t t_imu = trajectory_->GetForcedFixedTime();

    integrate_start_time = t_imu > 0 ? t_imu : 0;
    imu_state.timestamp_ns = t_imu;

    trajectory_->GetIMUState(integrate_start_time, imu_state);
  }
  imu_state.bias = all_imu_bias_.rbegin()->second;
  imu_state.gravity = gravity_;

  // 3. IMU积分
  imu_state_estimator_->Propagate(imu_state, integrate_start_time,
                                  trajectory_->MaxTimeNs(FrameType::IMU));

  // 4. 更新时间参数
  time_param_.UpdateCurScan(_scan_time_min, _scan_time_max);
  UpdateImuDataDuringLIO();

  // 5. 优化轨迹
  InitTrajectoryWithPropagation();
}

void TrajectoryManager::PredictTrajectory(int64_t _scan_time_min,
                                          int64_t _scan_time_max) {
  if (imu_data_cache_.size() < 2) {
    AWARN_F("[TrajectoryManager] IMU data is empty !");
    return;
  }

  // 1. 扩展轨迹
  ExtendTrajectory(_scan_time_max);

  // 2. 更新时间参数
  time_param_.UpdateCurScan(_scan_time_min, _scan_time_max);
  UpdateImuDataDuringLIO();

  // 3. 优化轨迹
  InitTrajectoryWithPropagation(false);
}

void TrajectoryManager::UndistortFeatures(const TimedCloudData2& _feature_data,
                                          KRTPointCloudPtr& _undistorted_cloud,
                                          bool _convert_to_map_frame_flag) {
  size_t feature_size = _feature_data.cloud->size();
  _undistorted_cloud->clear();
  _undistorted_cloud->resize(feature_size);
  _undistorted_cloud->is_dense = true;

  SE3d target_pose_to_G_inv =
      SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

  if (!_convert_to_map_frame_flag) {
    target_pose_to_G_inv =
        trajectory_
            ->GetSensorPose(_feature_data.scan_start_timestamp_ns,
                            FrameType::LIDAR)
            .inverse();
  }

  for (size_t i = 0; i < feature_size; i++) {
    KRTPoint point = _feature_data.cloud->points[i];
    SE3d pose_lk_to_G = trajectory_->GetSensorPose(
        _feature_data.scan_start_timestamp_ns + point.timestamp_ns,
        FrameType::LIDAR);
    Eigen::Vector3d p_Lk(point.x, point.y, point.z);
    Eigen::Vector3d point_out;
    point_out = target_pose_to_G_inv * pose_lk_to_G * p_Lk;

    point.x = point_out(0);
    point.y = point_out(1);
    point.z = point_out(2);
    _undistorted_cloud->points[i] = point;
  }
}

void TrajectoryManager::ExtendTrajectory(int64_t _max_time_ns) {
  SE3d last_knot = trajectory_->getLastKnot();
  trajectory_->extendKnotsTo(_max_time_ns, last_knot);
}

void TrajectoryManager::UpdateImuDataDuringLIO() {
  int idx = trajectory_->GetCtrlIndex(time_param_.cur_scan[0]);

  int64_t t_min =
      trajectory_->MinTimeNs(FrameType::LIDAR) + trajectory_->getDtNs() * idx;
  if (t_min < 0) t_min = 0;
  // double t_min = std::min(tparam_.last_scan[1], tparam_.cur_scan[0]);
  int64_t t_max = trajectory_->MaxTimeNs(FrameType::LIDAR);

  AINFO_F("[TrajectoryManager] t_min: %.3f, t_max: %.3f", t_min * 1e-9,
          t_max * 1e-9);

  if (imu_data_cache_.front().timestamp_ns >
      trajectory_->MaxTimeNs(FrameType::IMU)) {
    AWARN_F("[TrajectoryManager] IMU data sync wrong. ");
  }

  // 顺序查找
  for (auto iter = imu_data_cache_.begin(); iter != imu_data_cache_.end();
       ++iter) {
    if (iter->timestamp_ns >= t_min) {
      if (iter->timestamp_ns >= t_max) {
        continue;
      }
      time_param_.lio_imu_idx[0] = std::distance(imu_data_cache_.begin(), iter);
      time_param_.lio_imu_time[0] = iter->timestamp_ns;
      break;
    }
  }

  // 倒序查找
  for (auto rter = imu_data_cache_.rbegin(); rter != imu_data_cache_.rend();
       ++rter) {
    if (rter->timestamp_ns < t_max) {
      time_param_.lio_imu_idx[1] =
          std::distance(imu_data_cache_.begin(), rter.base()) - 1;
      time_param_.lio_imu_time[1] = rter->timestamp_ns;
      break;
    }
  }
}

void TrajectoryManager::InitTrajectoryWithPropagation(
    bool _use_velocity_factor) {
  AINFO_F(
      "[InitTrajWithPropagation] imu data time < %.3f, %.3f >, index < %i , %i "
      ">",
      time_param_.lio_imu_time[0] * 1e-9, time_param_.lio_imu_time[1] * 1e-9,
      time_param_.lio_imu_idx[0], time_param_.lio_imu_idx[1]);
  AINFO_F(
      "[InitTrajWithPropagation] Trajectory activate time < %.3f >, max time < "
      "%.3f >",
      trajectory_->GetActiveTime() * 1e-9, trajectory_->MaxTimeNs() * 1e-9);

  TrajectoryEstimatorOptions option;
  option.lock_accel_bias = true;
  option.lock_gyro_bias = true;
  option.lock_gravity = true;

  TrajectoryEstimator estimator(trajectory_, option);

  int64_t opt_min_time = time_param_.lio_imu_time[0];

  if (LocatedInFirstSegment(opt_min_time)) {
    estimator.SetFixedIndex(trajectory_->N - 1);
    // estimator->AddStartTimePose(original_pose_);
  } else {
    estimator.SetFixedIndex(lidar_prior_ctrl_id_.first - 1);
    // estimator->SetKeyScanConstant(trajectory_->GetForcedFixedTime());
  }

  // 激光先验
  if (lidar_marg_info_) {
    estimator.AddMarginalizationFactor(lidar_marg_info_,
                                       lidar_marg_parameter_blocks_);
  }

  // IMU测量
  double* param_bg = all_imu_bias_.rbegin()->second.gyro_bias.data();
  double* param_ba = all_imu_bias_.rbegin()->second.accel_bias.data();

  for (int i = time_param_.lio_imu_idx[0]; i <= time_param_.lio_imu_idx[1];
       ++i) {
    estimator.AddIMUMeasurementAnalytic(imu_data_cache_.at(i), param_bg,
                                        param_ba, gravity_.data(),
                                        optimization_weight_->imu_info_vec_);
  }

  // IMU速度
  if (_use_velocity_factor) {
    const IMUState imu_odom = imu_state_estimator_->GetLatestState();

    estimator.AddGlobalVelocityMeasurement(
        imu_odom.timestamp_ns, imu_odom.velocity,
        optimization_weight_->global_velocity_weight_);
  }

  // 求解
  ceres::Solver::Summary summary = estimator.Solve(50, false);
  AINFO_F("[TrajectoryManager] %s", summary.BriefReport().c_str());
}

bool TrajectoryManager::LocatedInFirstSegment(int64_t _time_cur) {
  size_t knot_idx = trajectory_->GetCtrlIndex(_time_cur);
  if (knot_idx < SplineOrder)
    return true;
  else
    return false;
}

void TrajectoryManager::SetOriginalPose(const Eigen::Vector3d& _pos,
                                        const Eigen::Quaterniond& _rot) {
  original_pose_.position = _pos;
  original_pose_.orientation.setQuaternion(_rot);
}

void TrajectoryManager::SetInitialState(const IMUState& _imu_state) {
  gravity_ = _imu_state.gravity;

  all_imu_bias_[_imu_state.timestamp_ns] = _imu_state.bias;

  SetOriginalPose(_imu_state.position, _imu_state.orientation);

  // 控制点初始化
  SO3d R0(_imu_state.orientation);
  for (size_t i = 0; i < trajectory_->numKnots(); i++) {
    trajectory_->setKnotSO3(R0, i);
  }

  Eigen::Vector3d euler =
      utils::R2ypr(_imu_state.orientation.toRotationMatrix()) / 180.0 * M_PI;
  std::cout << "Initail State:\n";
  std::cout << "\t- timestamp: " << _imu_state.timestamp_ns << std::endl;
  std::cout << "\t- position: " << _imu_state.position.transpose() << std::endl;
  std::cout << "\t- euler: " << euler.transpose() << std::endl;
  std::cout << "\t- gravity: " << gravity_.transpose() << std::endl;
  std::cout << "\t- gyr bias: " << _imu_state.bias.gyro_bias.transpose()
            << std::endl;
  std::cout << "\t- acc bias: " << _imu_state.bias.accel_bias.transpose()
            << std::endl;
}

}  // namespace multi_sensor_mapping