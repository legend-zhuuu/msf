#include "multi_sensor_mapping/frontend/imu/imu_processor.h"

#include "multi_sensor_mapping/utils/so3_math.h"
#include "multi_sensor_mapping/utils/utils_eigen.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

ImuProcessor::ImuProcessor()
    : init_iter_num_(1),
      first_frame_flag_(true),
      imu_need_init_flag_(true),
      start_timestamp_(-1) {
  Q_ = process_noise_cov();
  cov_acc_ = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr_ = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_bias_acc_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  cov_bias_gyr_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  cov_gyr_scale_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  cov_acc_scale_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  mean_acc_ = Eigen::Vector3d(0, 0, -1);
  mean_gyr_ = Eigen::Vector3d(0, 0, 0);

  angvel_last_ = Eigen::Vector3d::Zero();
  translation_lidar2imu_ = Eigen::Vector3d::Zero();
  rotation_lidar2imu_ = Eigen::Matrix3d::Identity();
}

void ImuProcessor::Reset() {
  cov_acc_ = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr_ = Eigen::Vector3d(0.1, 0.1, 0.1);
  angvel_last_ = Eigen::Vector3d::Zero();

  init_iter_num_ = 1;
  imu_need_init_flag_ = true;
  init_iter_num_ = 1;

  imu_data_cache_.clear();
  imu_state_container_.clear();
}

void ImuProcessor::SetExtrinsic(const Eigen::Vector3d& _translation_lidar2imu,
                                const Eigen::Matrix3d& _rotation_lidar2imu) {
  translation_lidar2imu_ = _translation_lidar2imu;
  rotation_lidar2imu_ = _rotation_lidar2imu;
}

void ImuProcessor::SetGyrCov(const Eigen::Vector3d& _scaler) {
  cov_gyr_scale_ = _scaler;
}

void ImuProcessor::SetAccCov(const Eigen::Vector3d& _scaler) {
  cov_acc_scale_ = _scaler;
}

void ImuProcessor::SetGyrBiasCov(const Eigen::Vector3d& _b_g) {
  cov_bias_gyr_ = _b_g;
}

void ImuProcessor::SetAccBiasCov(const Eigen::Vector3d& _b_a) {
  cov_bias_acc_ = _b_a;
}

void ImuProcessor::Process(
    const TimedCloudData& _cloud, const std::deque<IMUData>& _imu_data_vec,
    esekfom::esekf<state_ikfom, 12, input_ikfom>& _kf_state,
    CloudTypePtr& _feature_undistort) {
  if (_imu_data_vec.empty()) {
    AWARN_F("[ImuProcessor] IMU data is empty");
    return;
  }

  if (imu_need_init_flag_) {
    ImuInit(_cloud, _imu_data_vec, _kf_state, init_iter_num_);

    last_imu_ = _imu_data_vec.back();

    state_ikfom imu_state = _kf_state.get_x();
    if (init_iter_num_ > MAX_INI_COUNT) {
      cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
      imu_need_init_flag_ = false;

      cov_acc_ = cov_acc_scale_;
      cov_gyr_ = cov_gyr_scale_;

      AINFO_F("[ImuProcessor] IMU initial done ");
      AINFO_F(
          "IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f "
          "%.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f "
          "%.8f %.8f",
          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2],
          mean_acc_.norm(), cov_bias_gyr_[0], cov_bias_gyr_[1],
          cov_bias_gyr_[2], cov_acc_[0], cov_acc_[1], cov_acc_[2], cov_gyr_[0],
          cov_gyr_[1], cov_gyr_[2]);
    }

    return;
  }

  UndistortFeature(_cloud, _imu_data_vec, _kf_state, _feature_undistort);
}

void ImuProcessor::ImuInit(
    const TimedCloudData& _cloud, const std::deque<IMUData>& _imu_data_vec,
    esekfom::esekf<state_ikfom, 12, input_ikfom>& _kf_state, int& _N) {
  Eigen::Vector3d cur_acc, cur_gyr;

  ros::NodeHandle nh("~");
  vel_pub = nh.advertise<nav_msgs::Odometry>("/imu_velocity", 10, true);

  if (first_frame_flag_) {
    Reset();
    _N = 1;
    first_frame_flag_ = false;
    cur_acc = _imu_data_vec.front().accel;
    cur_gyr = _imu_data_vec.front().gyro;
    mean_acc_ = cur_acc;
    mean_gyr_ = cur_gyr;
    first_lidar_time_ = _cloud.timestamp;
  }

  // 计算加速度和陀螺仪的均值和方差
  for (const auto& imu : _imu_data_vec) {
    cur_acc = imu.accel;
    cur_gyr = imu.gyro;

    mean_acc_ += (cur_acc - mean_acc_) / _N;
    mean_gyr_ += (cur_gyr - mean_gyr_) / _N;

    cov_acc_ = cov_acc_ * (_N - 1.0) / _N +
               (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) *
                   (_N - 1.0) / (_N * _N);
    cov_gyr_ = cov_gyr_ * (_N - 1.0) / _N +
               (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) *
                   (_N - 1.0) / (_N * _N);
    _N++;
  }

  // 计算初始化角度
  Eigen::Vector3d z_axis = mean_acc_ / mean_acc_.norm();
  Eigen::Vector3d e_1(1, 0, 0);
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();
  Eigen::Matrix<double, 3, 1> y_axis = Eigen::SkewSymmetric(z_axis) * x_axis;

  // From these axes get rotation
  Eigen::Matrix<double, 3, 3> Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  Eigen::Matrix3d imu_rot_in_G = Ro.inverse();

  // 初始化重力、bias和外参
  state_ikfom init_state = _kf_state.get_x();
  init_state.rot = SO3(imu_rot_in_G);
  // init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * G_m_s2);
  init_state.grav = S2(Eigen::Vector3d(0, 0, -1) * G_m_s2);
  init_state.bg = mean_gyr_;
  init_state.offset_T_L_I = translation_lidar2imu_;
  init_state.offset_R_L_I = rotation_lidar2imu_;
  _kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = _kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  _kf_state.change_P(init_P);
  last_imu_ = _imu_data_vec.back();

  // 打印初始化角度
  double roll, pitch, yaw;
  utils::RoationMatrixd2YPR(imu_rot_in_G, yaw, pitch, roll);
  AINFO_F(
      "[ImuProcessor] IMU initial angle roll <%.4f> pitch <%.4f> yaw <%.4f> , "
      "time < %.4f >",
      roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI,
      _imu_data_vec.front().timestamp);
}

void ImuProcessor::UndistortFeature(
    const TimedCloudData& _cloud, const std::deque<IMUData>& _imu_data_vec,
    esekfom::esekf<state_ikfom, 12, input_ikfom>& _kf_state,
    CloudTypePtr& _feature_undistort) {
  _feature_undistort->clear();
  _feature_undistort->reserve(_cloud.cloud->size());

  auto v_imu = _imu_data_vec;
  v_imu.push_front(last_imu_);

  const double& imu_beg_time = v_imu.front().timestamp;
  const double& imu_end_time = v_imu.back().timestamp;
  const double& cloud_beg_time = _cloud.timestamp;
  const double& cloud_end_time =
      _cloud.cloud->points.back().time + _cloud.timestamp;

  // 初始化IMU状态
  state_ikfom imu_state = _kf_state.get_x();
  imu_state_container_.clear();
  imu_state_container_.reserve(v_imu.size());

  imu_state_container_.push_back(
      IMUState6D(0, imu_state.pos, imu_state.rot.toRotationMatrix(),
                 acc_s_last_, angvel_last_, imu_state.vel));

  // 前向推导
  Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  Eigen::Matrix3d R_imu;

  double dt = 0;
  input_ikfom in;
  for (auto iter_imu = v_imu.begin(); iter_imu < (v_imu.end() - 1);
       iter_imu++) {
    auto&& head = *(iter_imu);
    auto&& tail = *(iter_imu + 1);

    if (tail.timestamp < last_lidar_end_time_) continue;

    angvel_avr = 0.5 * (head.gyro + tail.gyro);
    acc_avr = 0.5 * (head.accel + tail.accel);

    acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();

    if (head.timestamp < last_lidar_end_time_) {
      dt = tail.timestamp - last_lidar_end_time_;
    } else {
      dt = tail.timestamp - head.timestamp;
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
    _kf_state.predict(dt, Q_, in);

    imu_state = _kf_state.get_x();

    angvel_last_ = angvel_avr - imu_state.bg;
    acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
      acc_s_last_[i] += imu_state.grav[i];
    }

    double&& offs_t = tail.timestamp - cloud_beg_time;
    imu_state_container_.push_back(
        IMUState6D(offs_t, imu_state.pos, imu_state.rot.toRotationMatrix(),
                   acc_s_last_, angvel_last_, imu_state.vel));
  }
  //publish velocity data.
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time().fromSec(imu_beg_time);
    odom.header.frame_id = "imu_link";

    //set the velocity
    odom.twist.twist.linear.x = imu_state.vel[0];
    odom.twist.twist.linear.y = imu_state.vel[1];
    odom.twist.twist.linear.z = imu_state.vel[2];

    //publish the message
    vel_pub.publish(odom);

  double note = cloud_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (cloud_end_time - imu_end_time);
  _kf_state.predict(dt, Q_, in);

  imu_state = _kf_state.get_x();

  last_imu_ = _imu_data_vec.back();
  last_lidar_end_time_ = cloud_end_time;

  // 反向推导
  if (_cloud.cloud->points.begin() == _cloud.cloud->points.end()) return;
  auto it_cloud = _cloud.cloud->points.end() - 1;

  int valid_cnt = 0;
  for (auto it_kp = imu_state_container_.end() - 1;
       it_kp != imu_state_container_.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;

    R_imu = head->rotation;
    vel_imu = head->vel;
    pos_imu = head->position;
    acc_imu = tail->acc;
    angvel_avr = tail->gyr;

    for (; it_cloud->time > head->offset_time; it_cloud--) {
      dt = it_cloud->time - head->offset_time;
      Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));

      Eigen::Vector3d P_i(it_cloud->x, it_cloud->y, it_cloud->z);
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
                           imu_state.pos);
      Eigen::Vector3d P_compensate =
          imu_state.offset_R_L_I.conjugate() *
          (imu_state.rot.conjugate() *
               (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) +
                T_ei) -
           imu_state.offset_T_L_I);

      PointType p;
      p.x = P_compensate(0);
      p.y = P_compensate(1);
      p.z = P_compensate(2);
      p.intensity = it_cloud->intensity;

      _feature_undistort->push_back(p);
      valid_cnt++;

      if (it_cloud == _cloud.cloud->points.begin()) break;
    }
  }

  _feature_undistort->resize(valid_cnt);
}

}  // namespace multi_sensor_mapping