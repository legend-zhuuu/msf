#ifndef MSM_ANALYTICAL_DIFF_LIDAR_FEATURE_FACTOR_H
#define MSM_ANALYTICAL_DIFF_LIDAR_FEATURE_FACTOR_H

#include <ceres/ceres.h>

#include "multi_sensor_mapping/factor/analytical_diff/split_spline_view.h"
#include "multi_sensor_mapping/spline/spline_segment.h"

namespace multi_sensor_mapping {

namespace analytic_derivative {

class PlanarFeatureFactor : public ceres::CostFunction,
                            So3SplineView,
                            RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  /**
   * @brief Construct a new Planar Feature Factor object 构造函数
   *
   * @param _point_timestamp
   * @param _measure_point
   * @param _geo_plane
   * @param _spline_segment_meta
   * @param _S_GtoM
   * @param _p_GinM
   * @param _S_LtoI
   * @param _p_LinI
   * @param _weight
   */
  PlanarFeatureFactor(
      int64_t _point_timestamp, const Eigen::Vector3d& _measure_point,
      const Eigen::Vector4d& _geo_plane,
      const SplineSegmentMeta<SplineOrder>& _spline_segment_meta,
      const SO3d& _S_GtoM, const Vec3d& _p_GinM, const SO3d& _S_LtoI,
      const Vec3d& _p_LinI, double _weight)
      : point_timestamp_(_point_timestamp),
        measure_point_(_measure_point),
        geo_plane_(_geo_plane),
        spline_segment_meta_(_spline_segment_meta),
        S_GtoM_(_S_GtoM),
        p_GinM_(_p_GinM),
        S_LtoI_(_S_LtoI),
        p_LinI_(_p_LinI),
        weight_(_weight) {
    set_num_residuals(1);
    size_t kont_num = spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    Vec3d p_Lk = measure_point_;
    Vec3d p_IK = S_LtoI_ * p_Lk + p_LinI_;

    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();

    size_t kont_num = spline_segment_meta_.NumParameters();
    if (jacobians) {
      S_ItoG = SO3View::EvaluateRp(point_timestamp_, spline_segment_meta_,
                                   parameters, &J_R);
      p_IinG = R3View::evaluate(point_timestamp_, spline_segment_meta_,
                                parameters + kont_num, &J_p);
    } else {
      S_ItoG = SO3View::EvaluateRp(point_timestamp_, spline_segment_meta_,
                                   parameters, nullptr);
      p_IinG = R3View::evaluate(point_timestamp_, spline_segment_meta_,
                                parameters + kont_num, nullptr);
    }
    Vec3d p_M = S_GtoM_ * (S_ItoG * p_IK + p_IinG) + p_GinM_;

    Vec3d J_pi = Vec3d::Zero();

    residuals[0] = p_M.transpose() * geo_plane_.head(3) + geo_plane_[3];

    J_pi = geo_plane_.head(3);

    residuals[0] *= weight_;

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < kont_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + kont_num]) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + kont_num]);
          jac_kont_p.setZero();
        }
      }
    }

    Mat3d J_Xm_R = -S_GtoM_.matrix() * S_ItoG.matrix() * SO3::hat(p_IK);
    Vec3d jac_lhs_R = Vec3d::Zero();
    Vec3d jac_lhs_P = Vec3d::Zero();

    jac_lhs_R = J_pi.transpose() * J_Xm_R;
    jac_lhs_P = J_pi.transpose() * S_GtoM_.matrix();

    /// Rotation control point
    for (size_t i = 0; i < kont_num; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 1*3 3*3
        jac_kont_R.block<1, 3>(0, 0) =
            jac_lhs_R.transpose() * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < kont_num; i++) {
      size_t idx = kont_num + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 1*3
        jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
        jac_kont_p = (weight_ * jac_kont_p).eval();
      }
    }
    return true;
  }

 private:
  /// @brief 点时间戳
  int64_t point_timestamp_;
  /// @brief 测量点
  Eigen::Vector3d measure_point_;
  /// @brief 平面方程
  Eigen::Vector4d geo_plane_;
  /// @brief 轨迹段元数据
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  /// @brief  重力系到机体系的旋转
  SO3d S_GtoM_;
  /// @brief 重力系到机体系的平移
  Vec3d p_GinM_;
  /// @brief 激光系到IMU的旋转
  SO3d S_LtoI_;
  /// @brief 激光系到IMU的平移
  Vec3d p_LinI_;
  /// @brief 权重
  double weight_;
};

}  // namespace analytic_derivative

}  // namespace multi_sensor_mapping

#endif