#include "multi_sensor_mapping/spline/trajectory_estimator.h"

#include "multi_sensor_mapping/factor/analytical_diff/lidar_feature_factor.h"
#include "multi_sensor_mapping/factor/analytical_diff/marginalization_factor.h"
#include "multi_sensor_mapping/factor/analytical_diff/trajectory_value_factor.h"
#include "multi_sensor_mapping/factor/auto_diff/trajectory_value_factor.h"
#include "multi_sensor_mapping/spline/ceres_local_param.h"
#include "multi_sensor_mapping/spline/trajectory.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

TrajectoryEstimator::TrajectoryEstimator(
    const std::shared_ptr<Trajectory>& _trajectory,
    const TrajectoryEstimatorOptions& _option)
    : options_(_option),
      trajectory_(_trajectory),
      fixed_control_point_index_(-1) {
  problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());
  if (_option.use_auto_diff) {
    auto_diff_local_parameterization_ = new LieLocalParameterization<SO3d>();
    analytic_local_parameterization_ = nullptr;
  } else {
    auto_diff_local_parameterization_ = nullptr;
    analytic_local_parameterization_ =
        new LieAnalyticLocalParameterization<SO3d>();
  }

  // For gravity
  homo_vec_local_parameterization_ =
      new ceres::HomogeneousVectorParameterization(3);

  marginalization_info_ = std::make_shared<MarginalizationInfo>();

  // 初始化时间偏置内存地址
  auto sensor_exp = trajectory_->GetSensorExtParams();
  for (auto& exp : sensor_exp) {
    time_offset_opt_params_[exp.first] = &exp.second.t_offset_ns;
  }
}

TrajectoryEstimator::~TrajectoryEstimator() {
  if (analytic_local_parameterization_) delete analytic_local_parameterization_;

  if (auto_diff_local_parameterization_)
    delete auto_diff_local_parameterization_;

  if (homo_vec_local_parameterization_) delete homo_vec_local_parameterization_;
}

void TrajectoryEstimator::SetKeyScanConstant(double _max_time) {
  int64_t time_ns;
  if (!MeasuredTimeToNs(LIDAR, _max_time, time_ns)) return;

  std::pair<double, size_t> max_i_s = trajectory_->computeTIndexNs(time_ns);

  int index = 0;
  if (max_i_s.first < 0.5) {
    index = max_i_s.second + SplineOrder - 2;
  } else {
    index = max_i_s.second + SplineOrder - 1;
  }
  if (fixed_control_point_index_ < index) {
    fixed_control_point_index_ = index;
  }

  AINFO_F(
      "[TrajectoryEstimator] Fixed control point index < %i , %i > , max time "
      "< %f >",
      fixed_control_point_index_, (int)trajectory_->numKnots(), _max_time);
}

bool TrajectoryEstimator::MeasuredTimeToNs(const FrameType& _frame_type,
                                           const double& _timestamp,
                                           int64_t& _time_ns) {
  _time_ns = _timestamp * S_TO_NS;

  return CheckMeasuredTime(_frame_type, _time_ns);
}

bool TrajectoryEstimator::CheckMeasuredTime(const FrameType& _frame_type,
                                            const int64_t& _timestamp_ns) {
  // 已考虑时间偏置
  int64_t t_min_traj_ns = trajectory_->MinTime(_frame_type) * S_TO_NS;
  int64_t t_max_traj_ns = trajectory_->MaxTime(_frame_type) * S_TO_NS;

  if (!options_.lock_ext_params.at(_frame_type).lock_t_offset) {
    // |____|______________________________|____|
    //    t_min                          t_max
    if (_timestamp_ns - options_.time_offset_padding_ns < t_min_traj_ns ||
        _timestamp_ns + options_.time_offset_padding_ns >= t_max_traj_ns)
      return false;
  } else {
    if (_timestamp_ns < t_min_traj_ns || _timestamp_ns >= t_max_traj_ns) {
      return false;
    }
  }

  return true;
}

void TrajectoryEstimator::SetFixedIndex(int _idx) {
  fixed_control_point_index_ = _idx;
}

int TrajectoryEstimator::GetFixedControlIndex() {
  return fixed_control_point_index_;
}

void TrajectoryEstimator::SetTimeoffsetState() {
  for (auto& sensor_t : time_offset_opt_params_) {
    if (problem_->HasParameterBlock(sensor_t.second)) {
      if (options_.lock_ext_params.at(sensor_t.first).lock_t_offset)
        problem_->SetParameterBlockConstant(sensor_t.second);
    }
  }
}

void TrajectoryEstimator::AddIMUPoseMeasurementAnalytic(
    const PoseData2& _pose_data, const Eigen::Matrix<double, 6, 1>& _info_vec) {
  int64_t time_ns = _pose_data.timestamp_ns;
  if (!CheckMeasuredTime(IMU, time_ns)) return;
  double* t_offset_ns = time_offset_opt_params_[IMU];
  int64_t t_corrected_ns = time_ns + (*t_offset_ns);

  // 检测时间优化是否锁定
  const auto& option_lock_ext = options_.lock_ext_params.at(IMU);

  SplineMeta<SplineOrder> spline_meta;
  if (option_lock_ext.lock_t_offset) {
    trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                    spline_meta);
  } else {
    trajectory_->CaculateSplineMeta(
        {{t_corrected_ns - options_.time_offset_padding_ns,
          t_corrected_ns + options_.time_offset_padding_ns}},
        spline_meta);
  }

  // 添加残差
  ceres::CostFunction* cost_function = new analytic_derivative::IMUPoseFactor(
      time_ns, _pose_data, spline_meta.segments.at(0), _info_vec);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);
  vec.push_back(t_offset_ns);  // time_offset
  problem_->AddResidualBlock(cost_function, NULL, vec);

  problem_->AddParameterBlock(t_offset_ns, 1);
  if (option_lock_ext.lock_t_offset) {
    problem_->SetParameterBlockConstant(t_offset_ns);
  } else {
    double t_ns = options_.time_offset_padding_ns;
    problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
    problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
  }
}

void TrajectoryEstimator::AddIMUMeasurementAnalytic(
    const IMUData2 _imu_data, double* _gyro_bias, double* _accel_bias,
    double* _gravity, const Eigen::Matrix<double, 6, 1>& _info_vec,
    bool _marg_this_factor) {
  int64_t time_ns = _imu_data.timestamp_ns;
  if (!CheckMeasuredTime(IMU, time_ns)) return;
  double* t_offset_ns = time_offset_opt_params_[IMU];
  int64_t t_corrected_ns = time_ns + (*t_offset_ns);

  const auto& option_lock_ext = options_.lock_ext_params.at(IMU);

  SplineMeta<SplineOrder> spline_meta;
  if (option_lock_ext.lock_t_offset) {
    trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                    spline_meta);
  } else {
    trajectory_->CaculateSplineMeta(
        {{t_corrected_ns - options_.time_offset_padding_ns,
          t_corrected_ns + options_.time_offset_padding_ns}},
        spline_meta);
  }

  // 添加残差
  ceres::CostFunction* cost_function = new analytic_derivative::IMUFactor(
      time_ns, _imu_data, spline_meta.segments.at(0), _info_vec);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);
  vec.emplace_back(_gyro_bias);
  vec.emplace_back(_accel_bias);
  vec.emplace_back(_gravity);
  vec.push_back(t_offset_ns);  // time_offset
  problem_->AddParameterBlock(_gravity, 3, homo_vec_local_parameterization_);

  if (options_.lock_gyro_bias) {
    problem_->AddParameterBlock(_gyro_bias, 3);
    problem_->SetParameterBlockConstant(_gyro_bias);
  }
  if (options_.lock_accel_bias) {
    problem_->AddParameterBlock(_accel_bias, 3);
    problem_->SetParameterBlockConstant(_accel_bias);
  }
  if (options_.lock_gravity) {
    problem_->SetParameterBlockConstant(_gravity);
  }

  problem_->AddParameterBlock(t_offset_ns, 1);
  if (option_lock_ext.lock_t_offset) {
    problem_->SetParameterBlockConstant(t_offset_ns);
  } else {
    double t_ns = options_.time_offset_padding_ns;
    problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
    problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
  }

  double cauchy_loss = _marg_this_factor ? 3 : 10;
  ceres::LossFunction* loss_function = NULL;
  // loss_function = new ceres::HuberLoss(1.0); // marg factor
  // 只支持柯西核函数
  loss_function = new ceres::CauchyLoss(cauchy_loss);  // adopt from vins-mono

  if (options_.is_marg_state && _marg_this_factor) {
    std::vector<int> drop_set_wo_ctrl_point;
    int Knot_size = 2 * spline_meta.NumParameters();
    // two bias
    if (options_.marg_bias_param) {
      drop_set_wo_ctrl_point.emplace_back(Knot_size);      // gyro_bias
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);  // accel_bias
    }
    if (options_.marg_gravity_param) {
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);  // gravity
    }
    if (options_.marg_t_offset_param)
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);  // t_offset
    PrepareMarginalizationInfo(RType_IMU, spline_meta, cost_function,
                               loss_function, vec, drop_set_wo_ctrl_point);
  } else {
    problem_->AddResidualBlock(cost_function, loss_function, vec);
  }
}

void TrajectoryEstimator::AddLidarMeasurementAnalytic(
    const PointCorrespondence& _lidar_measurement, const SO3d& _S_GtoM,
    const Eigen::Vector3d& _p_GinM, const SO3d& _S_LtoI,
    const Eigen::Vector3d& _p_LinI, double _weight, bool _marg_this_factor) {
  int64_t time_ns = _lidar_measurement.point_timestamp_ns;
  if (!CheckMeasuredTime(LIDAR, time_ns)) return;
  double* t_offset_ns = time_offset_opt_params_[LIDAR];
  int64_t t_corrected_ns = time_ns + (*t_offset_ns);

  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                  spline_meta);
  using Functor = analytic_derivative::PlanarFeatureFactor;
  ceres::CostFunction* cost_function = new Functor(
      time_ns, _lidar_measurement.point, _lidar_measurement.geo_plane,
      spline_meta.segments.at(0), _S_GtoM, _p_GinM, _S_LtoI, _p_LinI, _weight);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);

  if (options_.is_marg_state && _marg_this_factor) {
    int num_residuals = cost_function->num_residuals();
    Eigen::MatrixXd residuals;
    residuals.setZero(num_residuals, 1);

    cost_function->Evaluate(vec.data(), residuals.data(), nullptr);
    double dist = (residuals / _weight).norm();
    if (dist < 0.05) {
      std::vector<int> drop_set_wo_ctrl_point;
      PrepareMarginalizationInfo(RType_LiDAR, spline_meta, cost_function, NULL,
                                 vec, drop_set_wo_ctrl_point);
    }
  } else {
    problem_->AddResidualBlock(cost_function, NULL, vec);
  }
}

void TrajectoryEstimator::AddGlobalVelocityMeasurement(
    int64_t _time_ns, const Eigen::Vector3d& _velocity, double _weight) {
  if (!CheckMeasuredTime(IMU, _time_ns)) return;
  double* t_offset_ns = time_offset_opt_params_[IMU];
  int64_t t_corrected_ns = _time_ns + (*t_offset_ns);

  const auto& option_lock_ext = options_.lock_ext_params.at(IMU);

  SplineMeta<SplineOrder> spline_meta;
  if (option_lock_ext.lock_t_offset) {
    trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                    spline_meta);
  } else {
    trajectory_->CaculateSplineMeta(
        {{t_corrected_ns - options_.time_offset_padding_ns,
          t_corrected_ns + options_.time_offset_padding_ns}},
        spline_meta);
  }

  auto* cost_function = auto_diff::IMUGlobalVelocityFactor::Create(
      t_corrected_ns, _velocity, spline_meta, _weight);
  cost_function->AddParameterBlock(1);  // time_offset

  cost_function->SetNumResiduals(3);

  ceres::LossFunction* loss_function = NULL;
  // loss_function = new ceres::HuberLoss(1.0); // marg factor
  // 只支持柯西核函数
  loss_function = new ceres::CauchyLoss(60);  // adopt from vins-mono

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec, true);
  vec.push_back(t_offset_ns);  // time_offset
  problem_->AddResidualBlock(cost_function, loss_function, vec);

  problem_->AddParameterBlock(t_offset_ns, 1);
  if (option_lock_ext.lock_t_offset) {
    problem_->SetParameterBlockConstant(t_offset_ns);
  } else {
    double t_ns = options_.time_offset_padding_ns;
    problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
    problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
  }
}

void TrajectoryEstimator::AddMarginalizationFactor(
    const std::shared_ptr<MarginalizationInfo>& _last_marg_info,
    std::vector<double*>& _last_marginalization_parameter_blocks) {
  MarginalizationFactor* marginalization_factor =
      new MarginalizationFactor(_last_marg_info);
  problem_->AddResidualBlock(marginalization_factor, NULL,
                             _last_marginalization_parameter_blocks);
}

ceres::Solver::Summary TrajectoryEstimator::Solve(int _max_iterations,
                                                  bool _progress,
                                                  int _num_threads) {
  ceres::Solver::Options options;

  options.minimizer_type = ceres::TRUST_REGION;
  // options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  // options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //    options.trust_region_strategy_type = ceres::DOGLEG;
  //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  //    options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.minimizer_progress_to_stdout = _progress;

  if (_num_threads < 1) {
    _num_threads = 1;  // std::thread::hardware_concurrency(); // mine is 8
  }
  options.num_threads = _num_threads;
  options.max_num_iterations = _max_iterations;

  if (callbacks_.size() > 0) {
    for (auto& cb : callbacks_) {
      options.callbacks.push_back(cb.get());
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  // trajectory_->UpdateTimeOffset(t_offset_ns_opt_params_);

  return summary;
}

void TrajectoryEstimator::AddControlPoints(
    const SplineMeta<SplineOrder>& _spline_meta, std::vector<double*>& _vec,
    bool _add_pos_knot) {
  for (auto const& seg : _spline_meta.segments) {
    size_t start_idx = trajectory_->GetCtrlIndex(seg.t0_ns);
    for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i) {
      if (_add_pos_knot) {
        _vec.emplace_back(trajectory_->getKnotPos(i).data());
        problem_->AddParameterBlock(_vec.back(), 3);
      } else {
        _vec.emplace_back(trajectory_->getKnotSO3(i).data());
        if (options_.use_auto_diff) {
          problem_->AddParameterBlock(_vec.back(), 4,
                                      auto_diff_local_parameterization_);
        } else {
          problem_->AddParameterBlock(_vec.back(), 4,
                                      analytic_local_parameterization_);
        }
      }
      if (options_.lock_trajectory || (int)i <= fixed_control_point_index_) {
        problem_->SetParameterBlockConstant(_vec.back());
      }
    }
  }
}

void TrajectoryEstimator::PrepareMarginalizationInfo(
    ResidualType _r_type, ceres::CostFunction* _cost_function,
    ceres::LossFunction* _loss_function,
    std::vector<double*>& _parameter_blocks, std::vector<int>& _drop_set) {
  ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
      _r_type, _cost_function, NULL, _parameter_blocks, _drop_set);
  marginalization_info_->AddResidualBlockInfo(residual_block_info);
}

void TrajectoryEstimator::PrepareMarginalizationInfo(
    ResidualType _r_type, const SplineMeta<SplineOrder>& _spline_meta,
    ceres::CostFunction* _cost_function, ceres::LossFunction* _loss_function,
    std::vector<double*>& _parameter_blocks,
    std::vector<int>& _drop_set_wo_ctrl_point) {
  // add contrl point id to drop set
  std::vector<int> drop_set = _drop_set_wo_ctrl_point;
  if (options_.ctrl_to_be_opt_later > options_.ctrl_to_be_opt_now) {
    std::vector<int> ctrl_id;
    trajectory_->GetCtrlIdxs(_spline_meta, ctrl_id);
    for (int i = 0; i < (int)ctrl_id.size(); ++i) {
      if (ctrl_id[i] < options_.ctrl_to_be_opt_later) {
        drop_set.emplace_back(i);
        drop_set.emplace_back(i + _spline_meta.NumParameters());
      }
    }
  }

  // 对之后的优化没有约束的因子直接丢就行,因为留下来也没有约束作用
  if (drop_set.size() > 0) {
    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
        _r_type, _cost_function, _loss_function, _parameter_blocks, drop_set);
    marginalization_info_->AddResidualBlockInfo(residual_block_info);
  }
}

}  // namespace multi_sensor_mapping
