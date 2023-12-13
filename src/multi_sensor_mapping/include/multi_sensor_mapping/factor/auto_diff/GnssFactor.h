/* ----------------------------------------------------------------------------

* Copyright 2021, APRIL Lab,
* Hangzhou, Zhejiang, China
* All Rights Reserved
* Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

#ifndef MSM_AUTO_DIFF_GNSS_FACTOR_H
#define MSM_AUTO_DIFF_GNSS_FACTOR_H

#include <ceres/ceres.h>
#include <multi_sensor_mapping/utils/UtilsSensorData.h>

namespace multi_sensor_mapping {

/**
 * @brief The GNSSAlignmentADFactor class 6自由度
 */
class GNSSAlignmentADFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSAlignmentADFactor(Eigen::Vector3d _pose_in_map,
                        Eigen::Vector3d _pose_in_LENU)
      : pose_in_map_(_pose_in_map), pose_in_LENU_(_pose_in_LENU) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vec3T const> const p_map_in_LENU(param[0]);
    Eigen::Map<Quater const> const q_map_to_LENU(param[1]);

    Vec3T p_est =
        q_map_to_LENU * pose_in_map_.template cast<T>() + p_map_in_LENU;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = p_est - pose_in_LENU_.template cast<T>();
    return true;
  }

 private:
  /// @brief map坐标系的姿态
  Eigen::Vector3d pose_in_map_;
  /// @brief LENU坐标系的姿态
  Eigen::Vector3d pose_in_LENU_;
};

/**
 * @brief The GNSSAlignmentADFactor2 class 4自由度
 */
class GNSSAlignmentADFactor2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSAlignmentADFactor2(Eigen::Vector3d _pose_in_map,
                         Eigen::Vector3d _pose_in_LENU)
      : pose_in_map_(_pose_in_map), pose_in_LENU_(_pose_in_LENU) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vec3T const> const p_map_in_LENU(param[0]);

    T yaw_map_in_LENU = param[1][0];

    //    Eigen::Matrix<T, 3, 3> rot_mat;
    //    rot_mat << ceres::cos(yaw_map_in_LENU), ceres::sin(yaw_map_in_LENU),
    //    T(0),
    //        -ceres::sin(yaw_map_in_LENU), ceres::cos(yaw_map_in_LENU), T(0),
    //        T(0), T(0), T(1);

    Eigen::Quaternion<T> q(ceres::cos(yaw_map_in_LENU / T(2)), T(0), T(0),
                           ceres::sin(yaw_map_in_LENU / T(2)));

    Vec3T p_est = q * pose_in_map_.template cast<T>() + p_map_in_LENU;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = p_est - pose_in_LENU_.template cast<T>();
    return true;
  }

 private:
  /// @brief map坐标系的姿态
  Eigen::Vector3d pose_in_map_;
  /// @brief LENU坐标系的姿态
  Eigen::Vector3d pose_in_LENU_;
};

/**
 * @brief The GNSSAlignmentADFactor3 class 5自由度
 */
class GNSSAlignmentADFactor3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSAlignmentADFactor3(Eigen::Vector3d _pose_in_map,
                         Eigen::Vector3d _pose_in_LENU)
      : pose_in_map_(_pose_in_map), pose_in_LENU_(_pose_in_LENU) {}

  template <class T>
  bool operator()(T const* const* param, T* residuals) const {
    using Quater = Eigen::Quaternion<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vec3T const> const p_map_in_LENU(param[0]);

    T yaw_map_in_LENU = param[1][0];
    T pitch_map_in_LENU = param[2][0];

    //    Eigen::Matrix<T, 3, 3> rot_mat;
    //    rot_mat << ceres::cos(yaw_map_in_LENU), ceres::sin(yaw_map_in_LENU),
    //    T(0),
    //        -ceres::sin(yaw_map_in_LENU), ceres::cos(yaw_map_in_LENU), T(0),
    //        T(0), T(0), T(1);

    Eigen::Quaternion<T> q_yaw(ceres::cos(yaw_map_in_LENU / T(2)), T(0), T(0),
                               ceres::sin(yaw_map_in_LENU / T(2)));
    Eigen::Quaternion<T> q_pitch(ceres::cos(pitch_map_in_LENU / T(2)), T(0),
                                 ceres::sin(pitch_map_in_LENU / T(2)), T(0));

    Vec3T p_est =
        q_yaw * q_pitch * pose_in_map_.template cast<T>() + p_map_in_LENU;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = p_est - pose_in_LENU_.template cast<T>();
    return true;
  }

 private:
  /// @brief map坐标系的姿态
  Eigen::Vector3d pose_in_map_;
  /// @brief LENU坐标系的姿态
  Eigen::Vector3d pose_in_LENU_;
};
}  // namespace multi_sensor_mapping

#endif
