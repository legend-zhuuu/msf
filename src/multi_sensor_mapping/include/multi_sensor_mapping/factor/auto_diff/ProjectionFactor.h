/* ----------------------------------------------------------------------------

* Copyright 2021, APRIL Lab,
* Hangzhou, Zhejiang, China
* All Rights Reserved
* Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

#ifndef MSM_AUTO_DIFF_PROJECTION_FACTOR_H
#define MSM_AUTO_DIFF_PROJECTION_FACTOR_H

#include <ceres/ceres.h>

namespace multi_sensor_mapping {

class ProjectionADFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ProjectionADFactor(double obs_x, double obs_y, double fx, double fy,
                     double cx, double cy, double weight)
      : observed_x_(obs_x),
        observed_y_(obs_y),
        fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy),
        weight_(weight) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vec3T const> const camera_pos(param[0]);
    Eigen::Map<Quater const> const camera_quater(param[1]);
    Eigen::Map<Vec3T const> const map_point_pos(param[2]);

    Vec3T p_c = camera_quater.inverse() * (map_point_pos - camera_pos);

    const T xp = T(fx_) * p_c[0] / p_c[2] + T(cx_);
    const T yp = T(fy_) * p_c[1] / p_c[2] + T(cy_);

    Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residuals);
    res.template block<1, 1>(0, 0) = Eigen::Matrix<T, 1, 1>(xp - observed_x_);
    res.template block<1, 1>(1, 0) = Eigen::Matrix<T, 1, 1>(yp - observed_y_);

    res = T(weight_) * res;

    return true;
  }

 private:
  /// @brief 观测到的像素值
  double observed_x_;
  double observed_y_;

  /// @brief 相机内参
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  /// @brief 权重
  double weight_;
};

class Projection5DADFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Projection5DADFactor(double obs_x, double obs_y, double fx, double fy,
                       double cx, double cy, double weight, double pose_z)
      : observed_x_(obs_x),
        observed_y_(obs_y),
        fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy),
        weight_(weight),
        pose_z_(pose_z) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec2T = Eigen::Matrix<T, 2, 1>;

    Eigen::Map<Vec2T const> const camera_pos_xy(param[0]);
    Eigen::Map<Quater const> const camera_quater(param[1]);
    Eigen::Map<Vec3T const> const map_point_pos(param[2]);
    Vec3T camera_pos(camera_pos_xy(0), camera_pos_xy(1), T(pose_z_));

    Vec3T p_c = camera_quater.inverse() * (map_point_pos - camera_pos);

    const T xp = T(fx_) * p_c[0] / p_c[2] + T(cx_);
    const T yp = T(fy_) * p_c[1] / p_c[2] + T(cy_);

    Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residuals);
    res.template block<1, 1>(0, 0) = Eigen::Matrix<T, 1, 1>(xp - observed_x_);
    res.template block<1, 1>(1, 0) = Eigen::Matrix<T, 1, 1>(yp - observed_y_);

    res = T(weight_) * res;

    return true;
  }

 private:
  /// @brief 观测到的像素值
  double observed_x_;
  double observed_y_;

  /// @brief 相机内参
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  /// @brief 权重
  double weight_;

  double pose_z_;
};

}  // namespace multi_sensor_mapping
#endif
