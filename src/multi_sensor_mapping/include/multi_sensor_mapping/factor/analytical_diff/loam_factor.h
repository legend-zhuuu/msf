
#ifndef MSM_ANALYTICAL_DIFF_LOAM_FACTOR_H
#define MSM_ANALYTICAL_DIFF_LOAM_FACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "multi_sensor_mapping/utils/utils_eigen.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"

namespace multi_sensor_mapping {

/**
 * @brief The EdgeAnalyticFunctor class 边缘解析求导
 * Adepted from FLOAM
 */
class EdgeAnalyticFunctor : public ceres::SizedCostFunction<1, 7> {
 public:
  EdgeAnalyticFunctor(Eigen::Vector3d _curr_point,
                      Eigen::Vector3d _last_point_a,
                      Eigen::Vector3d _last_point_b)
      : curr_point_(_curr_point),
        last_point_a_(_last_point_a),
        last_point_b_(_last_point_b) {}

  virtual ~EdgeAnalyticFunctor() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);

    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point_ + t_last_curr;

    Eigen::Vector3d nu = (lp - last_point_a_).cross(lp - last_point_b_);
    Eigen::Vector3d de = last_point_a_ - last_point_b_;
    double de_norm = de.norm();
    residuals[0] = nu.norm() / de_norm;

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Matrix3d skew_lp = Eigen::SkewSymmetric(lp);
        Eigen::Matrix<double, 3, 6> dp_by_se3;
        dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
        (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(
            jacobians[0]);
        J_se3.setZero();
        Eigen::Matrix3d skew_de = Eigen::SkewSymmetric(de);
        J_se3.block<1, 6>(0, 0) =
            -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d last_point_a_;
  Eigen::Vector3d last_point_b_;
};

/**
 * @brief The SurfNormAnalyticFunctor class 平面点解析求导
 * Adapted from floam
 */
class SurfNormAnalyticFunctor : public ceres::SizedCostFunction<1, 7> {
 public:
  SurfNormAnalyticFunctor(Eigen::Vector3d _curr_point,
                          Eigen::Vector3d _plane_norm,
                          double _negative_OA_dot_norm)
      : curr_point_(_curr_point),
        plane_norm_(_plane_norm),
        negative_OA_dot_norm_(_negative_OA_dot_norm) {}

  virtual ~SurfNormAnalyticFunctor() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point_ + t_w_curr;
    residuals[0] = plane_norm_.dot(point_w) + negative_OA_dot_norm_;

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Matrix3d skew_point_w = Eigen::SkewSymmetric(point_w);
        Eigen::Matrix<double, 3, 6> dp_by_se3;
        dp_by_se3.block<3, 3>(0, 0) = -skew_point_w;
        (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(
            jacobians[0]);
        J_se3.setZero();
        J_se3.block<1, 6>(0, 0) = plane_norm_.transpose() * dp_by_se3;
      }
    }
    return true;
  }

 private:
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d plane_norm_;
  double negative_OA_dot_norm_;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
 public:
  PoseSE3Parameterization() {}

  virtual ~PoseSE3Parameterization() {}

  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    utils::GetTransformFromSe3(
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta), delta_q, delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;

    return true;
  }

  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();

    return true;
  }

  virtual int GlobalSize() const { return 7; }

  virtual int LocalSize() const { return 6; }
};

}  // namespace multi_sensor_mapping

#endif
