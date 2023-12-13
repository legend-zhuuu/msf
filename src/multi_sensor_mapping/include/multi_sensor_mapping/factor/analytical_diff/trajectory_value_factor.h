#ifndef MSM_ANALYTICAL_DIFF_TRAJECTORY_VALUE_FACTOR_H
#define MSM_ANALYTICAL_DIFF_TRAJECTORY_VALUE_FACTOR_H

#include <ceres/ceres.h>

#include "multi_sensor_mapping/factor/analytical_diff/split_spline_view.h"
#include "multi_sensor_mapping/spline/spline_segment.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

namespace analytic_derivative {

class GravityFactor : public ceres::SizedCostFunction<3, 3> {
 public:
  GravityFactor(const Eigen::Vector3d& gravity,
                const Eigen::Vector3d& sqrt_info)
      : gravity_(gravity), sqrt_info_(sqrt_info) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Map<Eigen::Vector3d const> gravity_now(parameters[0]);

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = sqrt_info_.asDiagonal() * (gravity_now - gravity_);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_grav(
            jacobians[0]);
        jac_grav.setZero();
        jac_grav.block<3, 3>(0, 0) =
            sqrt_info_.asDiagonal() * Eigen::Matrix3d::Identity();
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d gravity_;
  Eigen::Vector3d sqrt_info_;
};

// bias_gyr_i, bias_gyr_j, bias_acc_i, bias_acc_j
class BiasFactor : public ceres::SizedCostFunction<6, 3, 3, 3, 3> {
 public:
  BiasFactor(double dt, const Eigen::Matrix<double, 6, 1>& sqrt_info) {
    // double sqrt_dt = std::sqrt(dt);
    // sqrt_info_.setZero();
    // sqrt_info_.diagonal() = sqrt_info / sqrt_dt;
    sqrt_info_.setZero();
    sqrt_info_.diagonal() = sqrt_info;
  }
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    Eigen::Map<Vec3d const> bias_gyr_i(parameters[0]);
    Eigen::Map<Vec3d const> bias_gyr_j(parameters[1]);
    Eigen::Map<Vec3d const> bias_acc_i(parameters[2]);
    Eigen::Map<Vec3d const> bias_acc_j(parameters[3]);

    Vec6d res;
    res.block<3, 1>(0, 0) = bias_gyr_j - bias_gyr_i;
    res.block<3, 1>(3, 0) = bias_acc_j - bias_acc_i;

    Eigen::Map<Vec6d> residual(residuals);
    residual = sqrt_info_ * res;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bg_i(
            jacobians[0]);
        jac_bg_i.setZero();
        jac_bg_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        jac_bg_i.applyOnTheLeft(sqrt_info_);
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bg_j(
            jacobians[1]);
        jac_bg_j.setZero();
        jac_bg_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jac_bg_j.applyOnTheLeft(sqrt_info_);
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba_i(
            jacobians[2]);
        jac_ba_i.setZero();
        jac_ba_i.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        jac_ba_i.applyOnTheLeft(sqrt_info_);
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba_j(
            jacobians[3]);
        jac_ba_j.setZero();
        jac_ba_j.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
        jac_ba_j.applyOnTheLeft(sqrt_info_);
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d acc_i_, acc_j_;
  Eigen::Vector3d gyr_i_, gyr_j_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

class IMUFactor : public ceres::CostFunction, SplitSpineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  IMUFactor(int64_t time_ns, const IMUData2& imu_data,
            const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
            const Vec6d& info_vec)
      : time_ns_(time_ns),
        imu_data_(imu_data),
        spline_segment_meta_(spline_segment_meta),

        info_vec_(info_vec) {
    /// [误差维度 6]: 角速度误差 3 + 加速度误差 3
    set_num_residuals(6);

    /// [参数维度 2N+2]: 旋转控制点 N 个 + 平移控制点 N 个 + bias 2 + gravity
    size_t knot_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(3);  // gyro bias
    mutable_parameter_block_sizes()->push_back(3);  // accel bias
    mutable_parameter_block_sizes()->push_back(3);  // gravity
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_rot_w;
    typename SO3View::JacobianStruct J_rot_a;
    typename R3View::JacobianStruct J_pos;
    Mat3d J_gravity;

    size_t knot_num = this->spline_segment_meta_.NumParameters();
    Eigen::Map<Vec3d const> gyro_bias(parameters[2 * knot_num]);
    Eigen::Map<Vec3d const> accel_bias(parameters[2 * knot_num + 1]);
    Eigen::Map<Vec3d const> gravity(parameters[2 * knot_num + 2]);
    double time_offset_in_ns = parameters[2 * knot_num + 3][0];

    int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
    typename SplitView::SplineIMUData spline_data;
    if (jacobians) {
      spline_data =
          SplitView::Evaluate(t_corrected, spline_segment_meta_, parameters,
                              gravity, &J_rot_w, &J_rot_a, &J_pos);
    } else {
      spline_data = SplitView::Evaluate(t_corrected, spline_segment_meta_,
                                        parameters, gravity);
    }

    Eigen::Map<Vec6d> residual(residuals);
    residual.block<3, 1>(0, 0) =
        spline_data.gyro - (imu_data_.gyro - gyro_bias);
    residual.block<3, 1>(3, 0) =
        spline_data.accel - (imu_data_.accel - accel_bias);

    residual = (info_vec_.asDiagonal() * residual).eval();

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    /// Rotation control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = i + spline_data.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_knot_R(
            jacobians[idx]);
        jac_knot_R.setZero();
        /// for gyro residual
        jac_knot_R.block<3, 3>(0, 0) = J_rot_w.d_val_d_knot[i];
        /// for accel residual
        jac_knot_R.block<3, 3>(3, 0) = J_rot_a.d_val_d_knot[i];

        jac_knot_R = (info_vec_.asDiagonal() * jac_knot_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = knot_num + i + spline_data.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_knot_p(
            jacobians[idx]);
        jac_knot_p.setZero();

        /// for accel residual
        jac_knot_p.block<3, 3>(3, 0) =
            J_pos.d_val_d_knot[i] * spline_data.R_inv.matrix();
        jac_knot_p = (info_vec_.asDiagonal() * jac_knot_p).eval();
      }
    }

    /// bias
    if (jacobians[2 * knot_num]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bw(
          jacobians[2 * knot_num]);
      jac_bw.setZero();
      jac_bw.block<3, 3>(0, 0).diagonal() = info_vec_.head(3);
    }

    if (jacobians[2 * knot_num + 1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba(
          jacobians[2 * knot_num + 1]);
      jac_ba.setZero();
      jac_ba.block<3, 3>(3, 0).diagonal() = info_vec_.tail(3);
    }

    /// gravity
    if (jacobians[2 * knot_num + 2]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_gravity(
          jacobians[2 * knot_num + 2]);
      jac_gravity.setZero();
      jac_gravity.block<3, 3>(3, 0) = spline_data.R_inv.matrix();
      jac_gravity = (info_vec_.asDiagonal() * jac_gravity).eval();
    }

    // Time_offset
    if (jacobians[2 * knot_num + 3]) {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jac_t_offset(
          jacobians[2 * knot_num + 3]);
      jac_t_offset.setZero();

      Eigen::Vector3d rot_accel = SO3View::accelerationBody(
          t_corrected, spline_segment_meta_, parameters);
      Eigen::Vector3d jerk_R3 = R3View::evaluate<3>(
          t_corrected, spline_segment_meta_, parameters + knot_num);
      Eigen::Matrix3d rot = spline_data.R_inv.inverse().matrix();

      Eigen::Matrix3d gyro_hat = SO3d::hat(spline_data.gyro);
      Eigen::Matrix3d rot_dot = rot * gyro_hat;

      jac_t_offset.block<3, 1>(0, 0) =
          SO3d::vee(rot_dot.transpose() * rot_dot) +
          SO3d::vee(rot.transpose() *
                    (rot_dot * gyro_hat + rot * SO3d::hat(rot_accel)));
      jac_t_offset.block<3, 1>(3, 0) =
          rot_dot.transpose() * rot * spline_data.accel +
          rot.transpose() * jerk_R3;
      jac_t_offset = (1e-9 * info_vec_.asDiagonal() * jac_t_offset).eval();
    }

    return true;
  }

 private:
  int64_t time_ns_;
  IMUData2 imu_data_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  Vec6d info_vec_;
};

class IMUFactorNURBS : public ceres::CostFunction, SplitSpineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  IMUFactorNURBS(int64_t time_ns, const IMUData2& imu_data,
                 const Vec3d& gravity, const Vec6d& info_vec,
                 const std::vector<int64_t>& knts,
                 const std::pair<int, double>& su,
                 const Eigen::Matrix4d& blending_matrix,
                 const Eigen::Matrix4d& cumulative_blending_matrix)
      : time_ns_(time_ns),
        imu_data_(imu_data),
        gravity_(gravity),
        info_vec_(info_vec),
        knts_(knts),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix) {
    /// [误差维度 6]: 角速度误差 3 + 加速度误差 3
    set_num_residuals(6);
    /// [参数维度 2N+2]: 旋转控制点 N 个 + 平移控制点 N 个 + bias 2
    size_t knot_num = 4;
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(3);  // gyro bias
    mutable_parameter_block_sizes()->push_back(3);  // accel bias
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_rot_w;  // gyro residual对R导数
    typename SO3View::JacobianStruct J_rot_a;  // accel residual对R导数
    typename R3View::JacobianStruct J_pos;     // accel对P导数
    typename SplitView::SplineIMUData spline_data;

    if (jacobians) {
      spline_data = SplitView::EvaluateNURBS(
          time_ns_, parameters, gravity_, knts_, su_, blending_matrix_,
          cumulative_blending_matrix_, &J_rot_w, &J_rot_a, &J_pos);
    } else {
      spline_data = SplitView::EvaluateNURBS(time_ns_, parameters, gravity_,
                                             knts_, su_, blending_matrix_,
                                             cumulative_blending_matrix_);
    }

    size_t knot_num = 4;
    Eigen::Map<Vec3d const> gyro_bias(parameters[2 * knot_num]);
    Eigen::Map<Vec3d const> accel_bias(parameters[2 * knot_num + 1]);

    Eigen::Map<Vec6d> residual(residuals);
    residual.block<3, 1>(0, 0) =
        spline_data.gyro - (imu_data_.gyro - gyro_bias);
    residual.block<3, 1>(3, 0) =
        spline_data.accel - (imu_data_.accel - accel_bias);
    residual = (info_vec_.asDiagonal() * residual).eval();

    if (!jacobians) {
      return true;
    }
    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    /// Rotation control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_knot_R(
            jacobians[idx]);
        jac_knot_R.setZero();
        /// for gyro residual
        jac_knot_R.block<3, 3>(0, 0) = J_rot_w.d_val_d_knot[i];
        /// for accel residual
        jac_knot_R.block<3, 3>(3, 0) = J_rot_a.d_val_d_knot[i];

        jac_knot_R = (info_vec_.asDiagonal() * jac_knot_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = knot_num + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_knot_p(
            jacobians[idx]);
        jac_knot_p.setZero();

        /// for accel residual
        jac_knot_p.block<3, 3>(3, 0) =
            J_pos.d_val_d_knot[i] * spline_data.R_inv.matrix();
        jac_knot_p = (info_vec_.asDiagonal() * jac_knot_p).eval();
      }
    }

    /// bias
    if (jacobians[2 * knot_num]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bw(
          jacobians[2 * knot_num]);
      jac_bw.setZero();
      jac_bw.block<3, 3>(0, 0).diagonal() = info_vec_.head(3);
    }

    if (jacobians[2 * knot_num + 1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba(
          jacobians[2 * knot_num + 1]);
      jac_ba.setZero();
      jac_ba.block<3, 3>(3, 0).diagonal() = info_vec_.tail(3);
    }

    return true;
  }

 private:
  int64_t time_ns_;
  IMUData2 imu_data_;
  Vec3d gravity_;
  Vec6d info_vec_;
  std::vector<int64_t> knts_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
};

class IMUPoseFactor : public ceres::CostFunction {
 public:
  IMUPoseFactor(int64_t time_ns, const PoseData2& pose_data,
                const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
                const Eigen::Matrix<double, 6, 1>& info_vec)
      : time_ns_(time_ns),
        pose_data_(pose_data),
        spline_segment_meta_(spline_segment_meta),
        info_vec_(info_vec) {
    /// [误差维度 6]: 角度误差 3 + 平移误差 3
    set_num_residuals(6);
    /// [参数维度 2N]: 旋转控制点 N 个 + 平移控制点 N 个
    size_t kont_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;

    size_t knot_num = spline_segment_meta_.NumParameters();
    size_t P_offset = knot_num;

    double time_offset_in_ns = parameters[2 * knot_num][0];
    int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    if (jacobians) {
      S_ItoG = so3_spline_view.EvaluateRotation(
          t_corrected, spline_segment_meta_, parameters, &J_R);
      p_IinG = r3_spline_view.evaluate(t_corrected, spline_segment_meta_,
                                       parameters + P_offset, &J_p);
    } else {
      S_ItoG = so3_spline_view.EvaluateRotation(
          t_corrected, spline_segment_meta_, parameters);
      p_IinG = r3_spline_view.evaluate(t_corrected, spline_segment_meta_,
                                       parameters + P_offset);
    }

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) =
        (S_ItoG * pose_data_.orientation.inverse()).log();
    residual.block<3, 1>(3, 0) = p_IinG - pose_data_.position;

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix<double, 3, 1> res = residual.block<3, 1>(0, 0);
      Eigen::Matrix3d Jrot;
      Sophus::leftJacobianInvSO3(res, Jrot);

      for (size_t i = 0; i < SplineOrder; i++) {
        // 没有 timeoffset, J_R.start_idx 应该为 0, knot_num == N
        size_t idx = J_R.start_idx + i;
        if (jacobians[idx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>
              jacobian_kont_R(jacobians[idx]);
          jacobian_kont_R.setZero();
          /// for rotation residual
          jacobian_kont_R.block<3, 3>(0, 0) = Jrot * J_R.d_val_d_knot[i];
          /// L*J
          jacobian_kont_R = (info_vec_.asDiagonal() * jacobian_kont_R).eval();
        }
      }

      for (size_t i = 0; i < SplineOrder; i++) {
        size_t idx = J_R.start_idx + i;
        if (jacobians[idx + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>
              jacobian_kont_P(jacobians[idx + knot_num]);
          jacobian_kont_P.setZero();

          /// for position residual
          jacobian_kont_P.block<3, 3>(3, 0) =
              J_p.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
          /// L*J
          jacobian_kont_P = (info_vec_.asDiagonal() * jacobian_kont_P).eval();
        }
      }

      // Time_offset
      if (jacobians[2 * knot_num]) {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> jac_t_offset(
            jacobians[2 * knot_num]);
        jac_t_offset.setZero();

        Eigen::Vector3d gyro = So3SplineView::VelocityBody(
            t_corrected, spline_segment_meta_, parameters);
        Eigen::Vector3d vel = RdSplineView::velocity(
            t_corrected, spline_segment_meta_, parameters + knot_num);

        Eigen::Matrix3d gyro_hat = SO3d::hat(gyro);
        SO3d rot_dot = S_ItoG * SO3d(Eigen::Quaterniond(gyro_hat).normalized());
        jac_t_offset.block<3, 1>(0, 0) =
            (rot_dot * pose_data_.orientation.inverse()).log();  //

        jac_t_offset.block<3, 1>(3, 0) = vel;
        jac_t_offset = (1e-9 * info_vec_.asDiagonal() * jac_t_offset).eval();
      }
    }

    residual = (info_vec_.asDiagonal() * residual).eval();
    return true;
  }

 private:
  int64_t time_ns_;
  PoseData2 pose_data_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  Eigen::Matrix<double, 6, 1> info_vec_;
};

class LocalVelocityFactor : public ceres::CostFunction,
                            So3SplineView,
                            RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using SO3d = Sophus::SO3<double>;

  LocalVelocityFactor(int64_t time_ns, const Eigen::Vector3d& local_velocity,
                      const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
                      double weight)
      : time_ns_(time_ns),
        local_velocity_(local_velocity),
        spline_segment_meta_(spline_segment_meta),
        weight_(weight) {
    /// 线速度误差 3
    set_num_residuals(3);

    /// [参数维度 2N]: 旋转控制点 N 个 + 平移控制点 N 个
    size_t knot_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    SO3d S_ItoG;
    Vec3d v_inG = Vec3d::Zero();

    size_t knot_num = spline_segment_meta_.NumParameters();
    double time_offset_in_ns = parameters[2 * knot_num][0];

    int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;

    if (jacobians) {
      S_ItoG = SO3View::EvaluateRp(t_corrected, spline_segment_meta_,
                                   parameters, &J_R);
      v_inG = R3View::velocity(t_corrected, spline_segment_meta_,
                               parameters + knot_num, &J_p);
    } else {
      S_ItoG = SO3View::EvaluateRp(t_corrected, spline_segment_meta_,
                                   parameters, nullptr);
      v_inG = R3View::velocity(t_corrected, spline_segment_meta_,
                               parameters + knot_num, nullptr);
    }

    Eigen::Map<Vec3d> residual(residuals);
    residual = S_ItoG * local_velocity_ - v_inG;

    residual = (weight_ * residual).eval();

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    Eigen::Matrix3d jac_lhs_R;
    jac_lhs_R = -S_ItoG.matrix() * SO3::hat(local_velocity_);

    /// Rotation control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = i + J_R.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 3*3 3*3
        jac_kont_R.block<3, 3>(0, 0) = jac_lhs_R * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    Eigen::Matrix3d jac_lhs_P = -1 * Eigen::Matrix3d::Identity();
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = knot_num + i + J_p.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 3*3
        jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
        jac_kont_p = (weight_ * jac_kont_p).eval();
      }
    }

    // Time_offset
    if (jacobians[2 * knot_num]) {
      Eigen::Map<Eigen::Matrix<double, 3, 1>> jac_t_offset(
          jacobians[2 * knot_num]);
      jac_t_offset.setZero();

      Eigen::Vector3d gyro = So3SplineView::VelocityBody(
          t_corrected, spline_segment_meta_, parameters);
      Eigen::Vector3d accel = RdSplineView::acceleration(
          t_corrected, spline_segment_meta_, parameters + knot_num);

      Eigen::Matrix3d gyro_hat = SO3d::hat(gyro);
      Eigen::Matrix3d rot_dot = S_ItoG.matrix() * gyro_hat;

      jac_t_offset = rot_dot * local_velocity_ - accel;
      jac_t_offset = (1e-9 * weight_ * jac_t_offset).eval();
    }

    return true;
  }

 private:
  int64_t time_ns_;
  Eigen::Vector3d local_velocity_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  double weight_;
};

class Local6DoFVelocityFactor : public ceres::CostFunction,
                                So3SplineView,
                                RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using SO3d = Sophus::SO3<double>;

  Local6DoFVelocityFactor(
      int64_t time_ns, const Eigen::Matrix<double, 6, 1>& local_velocity,
      const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
      const Eigen::Matrix<double, 6, 1>& sqrt_info)
      : time_ns_(time_ns),
        local_velocity_(local_velocity),
        spline_segment_meta_(spline_segment_meta),
        sqrt_info_(sqrt_info) {
    /// 角速度+线速度 [6]
    set_num_residuals(6);

    /// [参数维度 2N]: 旋转控制点 N 个 + 平移控制点 N 个
    size_t knot_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    return false;
  }

 private:
  int64_t time_ns_;
  Eigen::Matrix<double, 6, 1> local_velocity_;

  SplineSegmentMeta<SplineOrder> spline_segment_meta_;

  Eigen::Matrix<double, 6, 1> sqrt_info_;
};

// class PreIntegrationFactor : public ceres::CostFunction,
//                              So3SplineView,
//                              RdSplineView {
//  public:
//   using SO3View = So3SplineView;
//   using R3View = RdSplineView;

//   using Mat3d = Eigen::Matrix<double, 3, 3>;
//   using Vec3d = Eigen::Matrix<double, 3, 1>;
//   using Mat15d = Eigen::Matrix<double, 15, 15>;
//   using SO3d = Sophus::SO3<double>;

//   PreIntegrationFactor(IntegrationBase* pre_integration, int64_t ta_ns,
//                        int64_t tb_ns,
//                        const SplineMeta<SplineOrder>& spline_meta)
//       : pre_integration_(pre_integration),
//         ta_ns_(ta_ns),
//         tb_ns_(tb_ns),
//         spline_meta_(spline_meta) {
//     /// [误差维度 3*5]
//     set_num_residuals(15);

//     /// [参数个数 2N+2]: 旋转控制点 N 个 + 平移控制点 N 个 + bias 4 个
//     size_t knot_num = spline_meta_.NumParameters();
//     for (size_t i = 0; i < knot_num; ++i) {
//       mutable_parameter_block_sizes()->push_back(4);
//     }
//     for (size_t i = 0; i < knot_num; ++i) {
//       mutable_parameter_block_sizes()->push_back(3);
//     }
//     mutable_parameter_block_sizes()->push_back(3);  // gyro bias(ta)
//     mutable_parameter_block_sizes()->push_back(3);  // gyro bias(tb)
//     mutable_parameter_block_sizes()->push_back(3);  // accel bias(ta)
//     mutable_parameter_block_sizes()->push_back(3);  // accel bias(tb)
//   }

//   virtual bool Evaluate(double const* const* parameters, double* residuals,
//                         double** jacobians) const {
//     typename SO3View::JacobianStruct J_R[2];
//     typename R3View::JacobianStruct J_p[2];
//     typename R3View::JacobianStruct J_v[2];

//     size_t Knot_offset = 2 * spline_meta_.NumParameters();
//     size_t kont_num = spline_meta_.NumParameters();

//     size_t R_offset[2] = {0, 0};
//     size_t P_offset[2] = {0, 0};
//     size_t seg_idx[2] = {0, 0};
//     {
//       double u;
//       spline_meta_.ComputeSplineIndex(ta_ns_, R_offset[0], u);
//       spline_meta_.ComputeSplineIndex(tb_ns_, R_offset[1], u);

//       // 默认最多只有两个 segments
//       size_t segment0_knot_num = spline_meta_.segments.at(0).NumParameters();
//       for (int i = 0; i < 2; ++i) {
//         if (R_offset[i] >= segment0_knot_num) {
//           seg_idx[i] = 1;
//           R_offset[i] = segment0_knot_num;
//         } else {
//           R_offset[i] = 0;
//         }
//         P_offset[i] = R_offset[i] + kont_num;
//       }
//     }

//     Eigen::Map<Vec3d const> gyro_bias_a_(parameters[Knot_offset]);
//     Eigen::Map<Vec3d const> gyro_bias_b_(parameters[Knot_offset + 1]);
//     Eigen::Map<Vec3d const> accel_bias_a_(parameters[Knot_offset + 2]);
//     Eigen::Map<Vec3d const> accel_bias_b_(parameters[Knot_offset + 3]);

//     Vec3d gyro_bias_a = gyro_bias_a_;
//     Vec3d gyro_bias_b = gyro_bias_b_;
//     Vec3d accel_bias_a = accel_bias_a_;
//     Vec3d accel_bias_b = accel_bias_b_;

//     SO3d S_IatoG, S_IbtoG;
//     SO3d S_GtoIa, S_GtoIb;
//     Vec3d p_IainG = Vec3d::Zero();
//     Vec3d p_IbinG = Vec3d::Zero();
//     Vec3d v_IainG = Vec3d::Zero();
//     Vec3d v_IbinG = Vec3d::Zero();

//     /// 计算 t_a 时刻 ti
//     if (jacobians) {
//       S_GtoIa =
//           SO3View::EvaluateRTp(ta_ns_, spline_meta_.segments.at(seg_idx[0]),
//                                parameters + R_offset[0], &J_R[0]);
//       p_IainG = R3View::evaluate(ta_ns_,
//       spline_meta_.segments.at(seg_idx[0]),
//                                  parameters + P_offset[0], &J_p[0]);
//       v_IainG = R3View::velocity(ta_ns_,
//       spline_meta_.segments.at(seg_idx[0]),
//                                  parameters + P_offset[0], &J_v[0]);

//       S_IbtoG =
//           SO3View::EvaluateRp(tb_ns_, spline_meta_.segments.at(seg_idx[1]),
//                               parameters + R_offset[1], &J_R[1]);
//       p_IbinG = R3View::evaluate(tb_ns_,
//       spline_meta_.segments.at(seg_idx[1]),
//                                  parameters + P_offset[1], &J_p[1]);
//       v_IbinG = R3View::velocity(tb_ns_,
//       spline_meta_.segments.at(seg_idx[1]),
//                                  parameters + P_offset[1], &J_v[1]);
//     } else {
//       S_GtoIa =
//           SO3View::EvaluateRTp(ta_ns_, spline_meta_.segments.at(seg_idx[0]),
//                                parameters + R_offset[0], nullptr);
//       p_IainG = R3View::evaluate(ta_ns_,
//       spline_meta_.segments.at(seg_idx[0]),
//                                  parameters + P_offset[0], nullptr);
//       v_IainG = R3View::velocity(ta_ns_,
//       spline_meta_.segments.at(seg_idx[0]),
//                                  parameters + P_offset[0], nullptr);

//       S_IbtoG =
//           SO3View::EvaluateRp(tb_ns_, spline_meta_.segments.at(seg_idx[1]),
//                               parameters + R_offset[1], nullptr);
//       p_IbinG = R3View::evaluate(tb_ns_,
//       spline_meta_.segments.at(seg_idx[1]),
//                                  parameters + P_offset[1], nullptr);
//       v_IbinG = R3View::velocity(tb_ns_,
//       spline_meta_.segments.at(seg_idx[1]),
//                                  parameters + P_offset[1], nullptr);
//     }
//     S_IatoG = S_GtoIa.inverse();
//     S_GtoIb = S_IbtoG.inverse();

//     Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
//     residual = pre_integration_->evaluate333(
//         p_IainG, S_IatoG, v_IainG, accel_bias_a, gyro_bias_a, p_IbinG,
//         S_IbtoG, v_IbinG, accel_bias_b, gyro_bias_b);

//     Vec3d res_R = residual.block<3, 1>(3, 0);

//     Mat15d sqrt_info =
//         Eigen::LLT<Mat15d>(pre_integration_->covariance.inverse())
//             .matrixL()
//             .transpose();
//     residual = sqrt_info * residual;

//     if (jacobians) {
//       for (size_t i = 0; i < kont_num; ++i) {
//         if (jacobians[i]) {
//           Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>>
//           jac_kont_R(
//               jacobians[i]);
//           jac_kont_R.setZero();
//         }
//         if (jacobians[i + kont_num]) {
//           Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//           jac_kont_p(
//               jacobians[i + kont_num]);
//           jac_kont_p.setZero();
//         }
//       }
//     }

//     if (jacobians) {
//       // [1] for positional residual
//       Mat3d jac_lhs_dp_dP[2];
//       Mat3d jac_lhs_dp_dR[2];
//       // [2] for rotational residual
//       Mat3d jac_lhs_dR_dR[2];
//       // [3] for velocity residual
//       Mat3d jac_lhs_dv_dP[2];
//       Mat3d jac_lhs_dv_dR[2];

//       // [1] for positional residual
//       jac_lhs_dp_dP[0] = -S_GtoIa.matrix();
//       jac_lhs_dp_dP[1] = S_GtoIa.matrix();

//       Vec3d gravity = Vec3d(0, 0, -GRAVITY_NORM);
//       double dt = pre_integration_->sum_dt;

//       // case(1) in EvaluateRTp();
//       // omitted left multiply term: R(t)^T * (rhs)_{\wedge}
//       Vec3d r_p_part =
//           p_IbinG - p_IainG - v_IainG * dt + 0.5 * dt * dt * gravity;
//       jac_lhs_dp_dR[0] = S_GtoIa.matrix() * SO3d::hat(r_p_part);
//       jac_lhs_dp_dR[1] = Mat3d::Zero();

//       // [2] for rotational residual
//       Eigen::Matrix3d Jrot;
//       Sophus::rightJacobianInvSO3(res_R, Jrot);
//       jac_lhs_dR_dR[0] = -Jrot * S_GtoIb.matrix();
//       jac_lhs_dR_dR[1] = Jrot;

//       // [3] for velocity residual
//       jac_lhs_dv_dP[0] = -S_GtoIa.matrix();
//       jac_lhs_dv_dP[1] = S_GtoIa.matrix();

//       // case(1) in EvaluateRTp();
//       jac_lhs_dv_dR[0] =
//           S_GtoIa.matrix() * SO3d::hat(v_IbinG - v_IainG + dt * gravity);
//       jac_lhs_dv_dR[1] = Mat3d::Zero();

//       Eigen::Matrix3d dp_dba =
//           pre_integration_->jacobian.block<3, 3>(O_P, O_BA);
//       Eigen::Matrix3d dv_dba =
//           pre_integration_->jacobian.block<3, 3>(O_V, O_BA);

//       Eigen::Matrix3d dp_dbg =
//           pre_integration_->jacobian.block<3, 3>(O_P, O_BG);
//       Eigen::Matrix3d dq_dbg =
//           pre_integration_->jacobian.block<3, 3>(O_R, O_BG);
//       Eigen::Matrix3d dv_dbg =
//           pre_integration_->jacobian.block<3, 3>(O_V, O_BG);

//       Mat3d jac_lhs_dbias[2];
//       jac_lhs_dbias[0] = -Mat3d::Identity();
//       jac_lhs_dbias[1] = Mat3d::Identity();
//       // ============================= TODO ============================= //
//       Mat3d jac_lhs_dR_dbg;
//       {
//         Vec3d delta_bg =
//             dq_dbg * (gyro_bias_a - pre_integration_->linearized_bg);
//         Mat3d Jbias;
//         Sophus::leftJacobianSO3(-delta_bg, Jbias);
//         Eigen::Matrix3d Jrot_r;
//         Sophus::leftJacobianInvSO3(res_R, Jrot_r);
//         jac_lhs_dR_dbg = -Jrot_r * Jbias * dq_dbg;
//       }

//       for (int seg = 0; seg < 2; ++seg) {
//         ///[1] Rotation control point
//         size_t pre_idx_R = R_offset[seg] + J_R[seg].start_idx;
//         for (size_t i = 0; i < SplineOrder; i++) {
//           size_t idx = pre_idx_R + i;
//           if (jacobians[idx]) {
//             Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>>
//                 jac_kont_R(jacobians[idx]);
//             ///[1.1] positional residual; ta(ti)
//             if (seg == 0) {
//               Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J_temp1;
//               // 3*3 3*3
//               J_temp1 = jac_lhs_dp_dR[seg] * J_R[seg].d_val_d_knot[i];
//               jac_kont_R.block<3, 3>(O_P, 0) += J_temp1;
//             }

//             ///[1.2] rotational residual
//             Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J_temp2;
//             // 3*3 3*3
//             J_temp2 = jac_lhs_dR_dR[seg] * J_R[seg].d_val_d_knot[i];
//             jac_kont_R.block<3, 3>(O_R, 0) += J_temp2;

//             ///[1.3] velocity residual; ta(ti)
//             if (seg == 0) {
//               Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J_temp3;
//               // 3*3 3*3
//               J_temp3 = jac_lhs_dv_dR[seg] * J_R[seg].d_val_d_knot[i];
//               jac_kont_R.block<3, 3>(O_V, 0) += J_temp3;
//             }
//           }
//         }

//         ///[2] position control point
//         size_t pre_idx_P = P_offset[seg] + J_p[seg].start_idx;
//         for (size_t i = 0; i < SplineOrder; i++) {
//           size_t idx = pre_idx_P + i;
//           if (jacobians[idx]) {
//             Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//                 jac_kont_p(jacobians[idx]);
//             ///[2.1] positional residual
//             Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J_temp1;
//             // 1*1 3*3
//             J_temp1 = J_p[seg].d_val_d_knot[i] * jac_lhs_dp_dP[seg];
//             if (seg == 0) {
//               J_temp1 += J_v[seg].d_val_d_knot[i] * jac_lhs_dp_dP[seg] *
//                          pre_integration_->sum_dt;
//             }
//             jac_kont_p.block<3, 3>(O_P, 0) += J_temp1;

//             ///[2.3] velocity residual
//             Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J_temp2;
//             // 1*1 3*3
//             J_temp2 = J_v[seg].d_val_d_knot[i] * jac_lhs_dv_dP[seg];
//             jac_kont_p.block<3, 3>(O_V, 0) += J_temp2;
//           }
//         }
//       }  // seg

//       ///[3] gyro bias ta
//       if (jacobians[Knot_offset]) {
//         Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//             jac_bias_gyro_ta(jacobians[Knot_offset]);
//         jac_bias_gyro_ta.setZero();
//         ///[3.1] positional residual
//         jac_bias_gyro_ta.block<3, 3>(O_P, 0) -= dp_dbg;
//         ///[3.2] rotational residual
//         jac_bias_gyro_ta.block<3, 3>(O_R, 0) += jac_lhs_dR_dbg;
//         ///[3.3] velocity residual
//         jac_bias_gyro_ta.block<3, 3>(O_V, 0) -= dv_dbg;
//         ///[3.5] gyro bias residual
//         jac_bias_gyro_ta.block<3, 3>(O_BG, 0) += jac_lhs_dbias[0];
//       }

//       ///[4] gyro bias tb
//       if (jacobians[Knot_offset + 1]) {
//         Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//             jac_bias_gyro_tb(jacobians[Knot_offset + 1]);
//         jac_bias_gyro_tb.setZero();
//         ///[4.5] gyro bias residual
//         jac_bias_gyro_tb.block<3, 3>(O_BG, 0) += jac_lhs_dbias[1];
//       }

//       ///[5] accel bias ta
//       if (jacobians[Knot_offset + 2]) {
//         Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//             jac_bias_accel_ta(jacobians[Knot_offset + 2]);
//         jac_bias_accel_ta.setZero();
//         ///[5.1] positional residual
//         jac_bias_accel_ta.block<3, 3>(O_P, 0) -= dp_dba;
//         ///[5.3] velocity residual
//         jac_bias_accel_ta.block<3, 3>(O_V, 0) -= dv_dba;
//         ///[5.4] accel bias residual
//         jac_bias_accel_ta.block<3, 3>(O_BA, 0) += jac_lhs_dbias[0];
//       }

//       ///[6] accel bias tb
//       if (jacobians[Knot_offset + 3]) {
//         Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//             jac_bias_accel_tb(jacobians[Knot_offset + 3]);
//         jac_bias_accel_tb.setZero();
//         ///[3.4] accel bias residual
//         jac_bias_accel_tb.block<3, 3>(O_BA, 0) += jac_lhs_dbias[1];
//       }

//       if (jacobians) {
//         for (size_t i = 0; i < kont_num; ++i) {
//           if (jacobians[i]) {
//             Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>>
//                 jac_kont_R(jacobians[i]);
//             jac_kont_R = (sqrt_info * jac_kont_R).eval();
//           }
//           if (jacobians[i + kont_num]) {
//             Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//                 jac_kont_p(jacobians[i + kont_num]);
//             jac_kont_p = (sqrt_info * jac_kont_p).eval();
//           }
//         }
//         for (size_t i = 0; i < 4; ++i) {
//           if (jacobians[i + Knot_offset]) {
//             Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
//                 jac_kont_bias(jacobians[i + Knot_offset]);
//             jac_kont_bias = (sqrt_info * jac_kont_bias).eval();
//           }
//         }
//       }
//     }

//     return true;
//   }

//   IntegrationBase* pre_integration_;
//   // ta < tb
//   int64_t ta_ns_;
//   int64_t tb_ns_;
//   SplineMeta<SplineOrder> spline_meta_;
// };

class RelativeOrientationFactor : public ceres::CostFunction, So3SplineView {
 public:
  using SO3View = So3SplineView;
  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using SO3d = Sophus::SO3<double>;

  RelativeOrientationFactor(const SO3d& S_BtoA, int64_t ta_ns, int64_t tb_ns,
                            const SplineMeta<SplineOrder>& spline_meta,
                            const Eigen::Vector3d& info_vec)
      : S_BtoA_(S_BtoA),
        ta_ns_(ta_ns),
        tb_ns_(tb_ns),
        spline_meta_(spline_meta) {
    /// [误差维度 3]
    set_num_residuals(3);

    /// [参数个数 N]: 旋转控制点 N 个
    size_t knot_num = spline_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }

    sqrt_info = Eigen::Matrix3d::Zero();
    sqrt_info.diagonal() = info_vec;
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R[2];

    size_t R_offset[2] = {0, 0};
    size_t seg_idx[2] = {0, 0};
    {
      double u;
      spline_meta_.ComputeSplineIndex(ta_ns_, R_offset[0], u);
      spline_meta_.ComputeSplineIndex(tb_ns_, R_offset[1], u);

      // 默认最多只有两个 segments
      size_t segment0_knot_num = spline_meta_.segments.at(0).NumParameters();
      for (int i = 0; i < 2; ++i) {
        if (R_offset[i] >= segment0_knot_num) {
          seg_idx[i] = 1;
          R_offset[i] = segment0_knot_num;
        } else {
          R_offset[i] = 0;
        }
      }
    }

    SO3d S_IatoG, S_GtoIb;
    if (jacobians) {
      S_IatoG = SO3View::EvaluateRotation(ta_ns_,
                                          spline_meta_.segments.at(seg_idx[0]),
                                          parameters + R_offset[0], &J_R[0]);
      S_GtoIb =
          SO3View::EvaluateLogRT(tb_ns_, spline_meta_.segments.at(seg_idx[1]),
                                 parameters + R_offset[1], &J_R[1]);
    } else {
      S_IatoG = SO3View::EvaluateRotation(ta_ns_,
                                          spline_meta_.segments.at(seg_idx[0]),
                                          parameters + R_offset[0], nullptr);
      S_GtoIb =
          SO3View::EvaluateLogRT(tb_ns_, spline_meta_.segments.at(seg_idx[1]),
                                 parameters + R_offset[1], nullptr);
    }

    Eigen::Map<Vec3d> residual(residuals);
    Vec3d res_R = (S_GtoIb * S_IatoG * S_BtoA_).log();
    residual.block<3, 1>(0, 0) = sqrt_info * res_R;

    size_t kont_num = spline_meta_.NumParameters();
    // initialize as zero
    if (jacobians) {
      for (size_t i = 0; i < kont_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix3d jac_lhs_dR_dR[2];
      Eigen::Matrix3d Jrot;
      Sophus::leftJacobianInvSO3(res_R, Jrot);
      jac_lhs_dR_dR[0] = Jrot * S_GtoIb.matrix();  // ta
      jac_lhs_dR_dR[1] = -Jrot;                    // tb

      for (int seg = 0; seg < 2; ++seg) {
        for (size_t i = 0; i < SplineOrder; i++) {
          size_t idx = R_offset[seg] + J_R[seg].start_idx + i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            // 3*3 3*3
            jac_kont_R.block<3, 3>(0, 0) +=
                sqrt_info * jac_lhs_dR_dR[seg] * J_R[seg].d_val_d_knot[i];
          }
        }
      }
    }

    return true;
  }

  Eigen::Matrix3d sqrt_info;

  SO3d S_BtoA_;

  double ta_ns_, tb_ns_;

  SplineMeta<SplineOrder> spline_meta_;
};

}  // namespace analytic_derivative

}  // namespace multi_sensor_mapping

#endif