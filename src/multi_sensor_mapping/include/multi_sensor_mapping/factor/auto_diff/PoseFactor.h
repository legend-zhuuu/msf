/* ----------------------------------------------------------------------------

* Copyright 2021, APRIL Lab,
* Hangzhou, Zhejiang, China
* All Rights Reserved
* Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

#ifndef MSM_AUTO_DIFF_POSE_FACTOR_H
#define MSM_AUTO_DIFF_POSE_FACTOR_H

#include <ceres/ceres.h>

namespace multi_sensor_mapping {

/**
 * @brief The AnchorPoseADFactor class 锚点姿态误差因子
 */
class AnchorPoseADFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AnchorPoseADFactor(Eigen::Vector3d anchor_position,
                     Eigen::Quaterniond anchor_orientation,
                     double position_weight, double orientation_weight)
      : anchor_position_(anchor_position),
        anchor_orientation_(anchor_orientation),
        position_weight_(position_weight),
        orientation_weight_(orientation_weight) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    Eigen::Map<Vec3T const> const camera_pos(param[0]);
    Eigen::Map<Quater const> const camera_quater(param[1]);

    Vec3T delta_p = camera_pos - anchor_position_.template cast<T>();
    Quater delta_q =
        anchor_orientation_.inverse().template cast<T>() * camera_quater;

    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
    res.template block<2, 1>(0, 0) =
        T(position_weight_) * delta_p.template block<2, 1>(0, 0);
    res.template block<1, 1>(2, 0) =
        Eigen::Matrix<T, 1, 1>(T(10000.0) * delta_p(2));
    res.template block<3, 1>(3, 0) =
        T(orientation_weight_) * T(2.0) * delta_q.vec();

    return true;
  }

 private:
  /// @brief 锚点位置
  Eigen::Vector3d anchor_position_;
  /// @brief 锚点旋转
  Eigen::Quaterniond anchor_orientation_;
  /// @brief 权重
  double position_weight_;
  double orientation_weight_;
};

class RelativePoseADFactpr {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RelativePoseADFactpr(Eigen::Vector3d relative_position,
                       Eigen::Quaterniond relative_orientation,
                       double position_weight, double orientation_weight)
      : relative_position_(relative_position),
        relative_orientation_(relative_orientation),
        position_weight_(position_weight),
        orientation_weight_(orientation_weight) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    Eigen::Map<Vec3T const> const camera_pos_a(param[0]);
    Eigen::Map<Quater const> const camera_quater_a(param[1]);

    Eigen::Map<Vec3T const> const camera_pos_b(param[0]);
    Eigen::Map<Quater const> const camera_quater_b(param[1]);

    Quater q_a_inverse = camera_quater_a.conjugate();
    Quater q_ab_estimate = q_a_inverse * camera_quater_b;

    Vec3T p_ab_estimate = q_a_inverse * (camera_pos_b - camera_pos_a);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
    res.template block<3, 1>(0, 0) =
        T(position_weight_) *
        (p_ab_estimate - relative_position_.template cast<T>());
    res.template block<3, 1>(3, 0) =
        T(orientation_weight_) * T(2.0) *
        (relative_orientation_.template cast<T>() * q_ab_estimate.conjugate())
            .vec();

    return true;
  }

 private:
  /// @brief 锚点位置
  Eigen::Vector3d relative_position_;
  /// @brief 锚点旋转
  Eigen::Quaterniond relative_orientation_;
  /// @brief 权重
  double position_weight_;
  double orientation_weight_;
};

}  // namespace multi_sensor_mapping

#endif
