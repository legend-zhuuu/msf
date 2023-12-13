#include "multi_sensor_mapping/frontend/imu/imu_pose_predictor.h"

#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

ImuPosePredictor::ImuPosePredictor(
    const std::shared_ptr<ExtrinsicParams> &_extrinsic_params,
    double _imu_gravity, double _imu_accel_noise, double _imu_gyro_noise,
    double _imu_accel_bias_noise, double _imu_gyro_bias_noise)
    : ImuPredictorBase(_extrinsic_params),
      imu_gravity_(_imu_gravity),
      imu_accel_noise_(_imu_accel_noise),
      imu_gyro_noise_(_imu_gyro_noise),
      imu_accel_bias_noise_(_imu_accel_bias_noise),
      imu_gyro_bias_noise_(_imu_gyro_bias_noise),
      initialized_flag_(false),
      key_(0),
      initial_bias_flag_(false),
      last_opt_imu_time_(-1) {
  InitGtsamVariable();
}

void ImuPosePredictor::AddImuData(const IMUData &_imu_data) {
  if (!imu_data_cache_.empty() &&
      _imu_data.timestamp <= imu_data_cache_.back().timestamp) {
    return;
  }
  imu_data_cache_.push_back(_imu_data);
  opt_imu_data_cache_.push_back(_imu_data);
}

void ImuPosePredictor::CorrectPose(double timestamp,
                                   const Eigen::Matrix4d &pose,
                                   bool is_degenerate, FrameType frame) {
  gtsam::Pose3 correction_pose = PoseToImuFrame(pose, frame);
  last_correction_time_ = timestamp;

  if (!initialized_flag_) {
    ResetOptimization();

    while (!opt_imu_data_cache_.empty()) {
      if (opt_imu_data_cache_.front().timestamp < timestamp) {
        last_opt_imu_time_ = opt_imu_data_cache_.front().timestamp;
        opt_imu_data_cache_.pop_front();
      } else
        break;
    }
    // 初始pose
    prev_pose_ = correction_pose;
    gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_,
                                                prior_pose_noise_);
    graph_factors_.add(prior_pose);
    // 初始速度
    prev_vel_ = gtsam::Vector3(0, 0, 0);
    gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_,
                                                 prior_vel_noise_);
    graph_factors_.add(prior_vel);
    //  初始bias
    if (!initial_bias_flag_) {
      prev_bias_ = gtsam::imuBias::ConstantBias();
    } else {
      initial_bias_flag_ = false;
    }
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(
        B(0), prev_bias_, prior_bias_noise_);
    graph_factors_.add(prior_bias);

    graph_values_.insert(X(0), prev_pose_);
    graph_values_.insert(V(0), prev_vel_);
    graph_values_.insert(B(0), prev_bias_);

    optimizer_.update(graph_factors_, graph_values_);
    graph_factors_.resize(0);
    graph_values_.clear();

    imu_integrator_->resetIntegrationAndSetBias(prev_bias_);
    imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
    key_ = 1;
    initialized_flag_ = true;
    return;
  }

  if (key_ == 100) {
    gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise =
        gtsam::noiseModel::Gaussian::Covariance(
            optimizer_.marginalCovariance(X(key_ - 1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise =
        gtsam::noiseModel::Gaussian::Covariance(
            optimizer_.marginalCovariance(V(key_ - 1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise =
        gtsam::noiseModel::Gaussian::Covariance(
            optimizer_.marginalCovariance(B(key_ - 1)));

    ResetOptimization();

    gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_,
                                                updatedPoseNoise);
    graph_factors_.add(prior_pose);
    prev_vel_ = gtsam::Vector3(0, 0, 0);
    gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_,
                                                 updatedVelNoise);
    graph_factors_.add(prior_vel);
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(
        B(0), prev_bias_, updatedBiasNoise);
    graph_factors_.add(prior_bias);

    graph_values_.insert(X(0), prev_pose_);
    graph_values_.insert(V(0), prev_vel_);
    graph_values_.insert(B(0), prev_bias_);

    optimizer_.update(graph_factors_, graph_values_);
    graph_factors_.resize(0);
    graph_values_.clear();
    key_ = 1;
  }

  // 积分IMU数据
  while (!opt_imu_data_cache_.empty()) {
    double imu_time = opt_imu_data_cache_.front().timestamp;
    if (imu_time < timestamp) {
      double dt = (last_opt_imu_time_ < 0) ? (1.0 / 500.0)
                                           : (imu_time - last_opt_imu_time_);
      imu_integrator_opt_->integrateMeasurement(
          opt_imu_data_cache_.front().accel, opt_imu_data_cache_.front().gyro,
          dt);
      last_opt_imu_time_ = imu_time;
      opt_imu_data_cache_.pop_front();
    } else
      break;
  }

  const gtsam::PreintegratedImuMeasurements &preint_imu =
      dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
          *imu_integrator_opt_);
  gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_),
                              B(key_ - 1), preint_imu);
  graph_factors_.add(imu_factor);
  graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(
          sqrt(imu_integrator_opt_->deltaTij()) * noise_model_between_bias_)));

  gtsam::PriorFactor<gtsam::Pose3> pose_factor(
      X(key_), correction_pose,
      is_degenerate ? correction_noise2_ : correction_noise_);
  graph_factors_.add(pose_factor);

  gtsam::NavState prop_state_ =
      imu_integrator_opt_->predict(prev_state_, prev_bias_);
  graph_values_.insert(X(key_), prop_state_.pose());
  graph_values_.insert(V(key_), prop_state_.v());
  graph_values_.insert(B(key_), prev_bias_);

  // 优化
  optimizer_.update(graph_factors_, graph_values_);
  optimizer_.update();
  graph_factors_.resize(0);
  graph_values_.clear();

  gtsam::Values result = optimizer_.calculateEstimate();
  prev_pose_ = result.at<gtsam::Pose3>(X(key_));
  prev_vel_ = result.at<gtsam::Vector3>(V(key_));
  prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
  prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));

  imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

  // 失败检测
  if (FailureDetection(prev_vel_, prev_bias_)) {
    AWARN_F("[ImuPosePredictor] Optimization Failed! Reset optimization！");
    initialized_flag_ = false;
    last_opt_imu_time_ = -1;
    return;
  }

  ++key_;
}

bool ImuPosePredictor::Predict(double timestamp, Eigen::Matrix4d &sensor_pose,
                               FrameType frame) {
  double last_imu_time = -1;
  // 删除太老的数据
  while (!imu_data_cache_.empty()) {
    if (imu_data_cache_.front().timestamp < last_correction_time_) {
      last_imu_time = imu_data_cache_.front().timestamp;
      imu_data_cache_.pop_front();
    } else
      break;
  }

  if (imu_data_cache_.empty()) {
    AWARN_F("[ImuPosePredictor] Please add enough IMU data first");
    return false;
  }

  // IMU积分
  imu_integrator_->resetIntegrationAndSetBias(prev_bias_);

  for (int i = 0; i < imu_data_cache_.size(); i++) {
    double imu_time = imu_data_cache_[i].timestamp;
    if (imu_time < timestamp) {
      double dt =
          (last_imu_time < 0) ? (1.0 / 500.0) : (imu_time - last_imu_time);
      imu_integrator_->integrateMeasurement(imu_data_cache_[i].accel,
                                            imu_data_cache_[i].gyro, dt);

      last_imu_time = imu_time;
    } else
      break;
  }
  //  while (!imu_data_cache_.empty()) {
  //    double imu_time = imu_data_cache_.front().timestamp;
  //    if (imu_time < timestamp) {
  //      double dt =
  //          (last_imu_time < 0) ? (1.0 / 500.0) : (imu_time - last_imu_time);
  //      imu_integrator_->integrateMeasurement(imu_data_cache_.front().accel,
  //                                            imu_data_cache_.front().gyro,
  //                                            dt);
  //      last_imu_time = imu_time;
  //      //      imu_data_cache_.pop_front();
  //    } else
  //      break;
  //  }

  gtsam::NavState cur_state = imu_integrator_->predict(prev_state_, prev_bias_);
  sensor_pose = ImuPoseToSensorFrame(cur_state.pose(), frame);
  return true;
}

void ImuPosePredictor::SetInitialBias(Eigen::Vector3d gyro_bias,
                                      Eigen::Vector3d accel_bias) {
  prev_bias_ = gtsam::imuBias::ConstantBias(accel_bias, gyro_bias);
  initial_bias_flag_ = true;
}

void ImuPosePredictor::InitGtsamVariable() {
  boost::shared_ptr<gtsam::PreintegrationParams> p =
      gtsam::PreintegrationParams::MakeSharedU(imu_gravity_);
  p->accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(imu_accel_noise_, 2);
  p->gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(imu_gyro_noise_, 2);
  p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);
  gtsam::imuBias::ConstantBias prior_imu_bias(
      (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

  prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2)
          .finished());  // rad,rad,rad,m, m, m
  prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
  prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigma(
      6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
  correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1)
          .finished());  // rad,rad,rad,m, m, m
  correction_noise2_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1, 1, 1, 1, 1, 1)
          .finished());  // rad,rad,rad,m, m, m
  noise_model_between_bias_ =
      (gtsam::Vector(6) << imu_accel_bias_noise_, imu_accel_bias_noise_,
       imu_accel_bias_noise_, imu_gyro_bias_noise_, imu_gyro_bias_noise_,
       imu_gyro_bias_noise_)
          .finished();

  imu_integrator_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
  imu_integrator_opt_ =
      new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
}

void ImuPosePredictor::ResetOptimization() {
  gtsam::ISAM2Params optParameters;
  optParameters.relinearizeThreshold = 0.1;
  optParameters.relinearizeSkip = 1;
  optimizer_ = gtsam::ISAM2(optParameters);

  gtsam::NonlinearFactorGraph newGraphFactors;
  graph_factors_ = newGraphFactors;

  gtsam::Values NewGraphValues;
  graph_values_ = NewGraphValues;
}

gtsam::Pose3 ImuPosePredictor::PoseToImuFrame(const Eigen::Matrix4d &pose,
                                              FrameType frame) {
  Eigen::Matrix4d pose_in_imu_frame = pose * extrinsic_matrix_[frame].inverse();
  Eigen::Quaterniond pose_q(pose_in_imu_frame.block<3, 3>(0, 0));
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(pose_q.w(), pose_q.x(), pose_q.y(), pose_q.z()),
      gtsam::Point3(pose_in_imu_frame(0, 3), pose_in_imu_frame(1, 3),
                    pose_in_imu_frame(2, 3)));
}

Eigen::Matrix4d ImuPosePredictor::ImuPoseToSensorFrame(const gtsam::Pose3 &pose,
                                                       FrameType frame) {
  Eigen::Matrix4d pose_imu_frame = pose.matrix();
  return pose_imu_frame * extrinsic_matrix_[frame];
}

bool ImuPosePredictor::FailureDetection(
    const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur) {
  Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
  if (vel.norm() > 30) {
    AWARN_F("[ImuPosePredictor] Large velocity! Reset IMU-preintegration!");

    return true;
  }

  Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                     biasCur.accelerometer().z());
  Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                     biasCur.gyroscope().z());
  if (ba.norm() > 1.0 || bg.norm() > 1.0) {
    AWARN_F("[ImuPosePredictor] Large bias! Reset IMU-preintegration!");

    return true;
  }

  return false;
}

}  // namespace multi_sensor_mapping
