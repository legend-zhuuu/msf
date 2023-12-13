#ifndef MSM_AUTO_DIFF_POSE_GRAPH_FACTOR_H
#define MSM_AUTO_DIFF_POSE_GRAPH_FACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace multi_sensor_mapping {

template <typename T>
inline void QuaternionInverse(const T q[4], T q_inverse[4]) {
  q_inverse[0] = q[0];
  q_inverse[1] = -q[1];
  q_inverse[2] = -q[2];
  q_inverse[3] = -q[3];
};

/**
 * @brief The RelativeRTFactor class
 * Adapted from M-LOAM
 */
class RelativeRTFactor {
 public:
  RelativeRTFactor(double t_x, double t_y, double t_z, double q_w, double q_x,
                   double q_y, double q_z, double t_var, double q_var)
      : t_x(t_x),
        t_y(t_y),
        t_z(t_z),
        q_w(q_w),
        q_x(q_x),
        q_y(q_y),
        q_z(q_z),
        t_var(t_var),
        q_var(q_var) {}

  template <typename T>
  bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j,
                  const T* tj, T* residuals) const {
    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    T i_q_w[4];
    QuaternionInverse(w_q_i, i_q_w);

    T t_i_ij[3];
    ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
    residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
    residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

    T relative_q[4];
    relative_q[0] = T(q_w);
    relative_q[1] = T(q_x);
    relative_q[2] = T(q_y);
    relative_q[3] = T(q_z);

    T q_i_j[4];
    ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

    T relative_q_inv[4];
    QuaternionInverse(relative_q, relative_q_inv);

    T error_q[4];
    ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

    residuals[3] = T(2) * error_q[1] / T(q_var);
    residuals[4] = T(2) * error_q[2] / T(q_var);
    residuals[5] = T(2) * error_q[3] / T(q_var);

    return true;
  }

  static ceres::CostFunction* Create(const double t_x, const double t_y,
                                     const double t_z, const double q_w,
                                     const double q_x, const double q_y,
                                     const double q_z, const double t_var,
                                     const double q_var) {
    return (new ceres::AutoDiffCostFunction<RelativeRTFactor, 6, 4, 3, 4, 3>(
        new RelativeRTFactor(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
  }

 private:
  double t_x, t_y, t_z;
  double q_w, q_x, q_y, q_z;
  double t_var, q_var;
};

/**
 * @brief The AnchorPointFactor class
 */
class AnchorPointFactor {
 public:
  AnchorPointFactor(double _gps_x, double _gps_y, double _gps_z,
                    double _noise_x, double _noise_y, double _noise_z)
      : gps_x(_gps_x),
        gps_y(_gps_y),
        gps_z(_gps_z),
        noise_x(_noise_x),
        noise_y(_noise_y),
        noise_z(_noise_z) {}

  template <typename T>
  bool operator()(const T* const w_q_i, const T* ti, const T* t_base_anchor,
                  T* residuals) const {
    // 外参
    T ex_p[3];
    ceres::QuaternionRotatePoint(w_q_i, t_base_anchor, ex_p);

    residuals[0] = (ti[0] + ex_p[0] - T(gps_x)) / T(noise_x);
    residuals[1] = (ti[1] + ex_p[1] - T(gps_y)) / T(noise_y);
    residuals[2] = (ti[2] + ex_p[2] - T(gps_z)) / T(noise_z);

    return true;
  }

  static ceres::CostFunction* Create(const double _gps_x, const double _gps_y,
                                     const double _gps_z, const double _noise_x,
                                     const double _noise_y,
                                     const double _noise_z) {
    return (new ceres::AutoDiffCostFunction<AnchorPointFactor, 3, 4, 3, 3>(
        new AnchorPointFactor(_gps_x, _gps_y, _gps_z, _noise_x, _noise_y,
                              _noise_z)));
  }

 private:
  double gps_x, gps_y, gps_z;
  double noise_x, noise_y, noise_z;
};

/**
 * @brief The AnchorPointFactor2 class
 */
class AnchorPointFactor2 {
 public:
  AnchorPointFactor2(double _gps_x, double _gps_y, double _noise_x,
                     double _noise_y)
      : gps_x(_gps_x), gps_y(_gps_y), noise_x(_noise_x), noise_y(_noise_y) {}

  template <typename T>
  bool operator()(const T* const w_q_i, const T* ti, const T* t_base_anchor,
                  T* residuals) const {
    // 外参
    T ex_p[3];
    ceres::QuaternionRotatePoint(w_q_i, t_base_anchor, ex_p);

    residuals[0] = (ti[0] + ex_p[0] - T(gps_x)) / T(noise_x);
    residuals[1] = (ti[1] + ex_p[1] - T(gps_y)) / T(noise_y);

    return true;
  }

  static ceres::CostFunction* Create(const double _gps_x, const double _gps_y,
                                     const double _noise_x,
                                     const double _noise_y) {
    return (new ceres::AutoDiffCostFunction<AnchorPointFactor2, 2, 4, 3, 3>(
        new AnchorPointFactor2(_gps_x, _gps_y, _noise_x, _noise_y)));
  }

 private:
  double gps_x, gps_y;
  double noise_x, noise_y;
};

}  // namespace multi_sensor_mapping

#endif
