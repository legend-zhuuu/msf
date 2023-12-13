#ifndef MSM_AUTO_DIFF_TRAJECTORY_VALUE_FACTOR_H
#define MSM_AUTO_DIFF_TRAJECTORY_VALUE_FACTOR_H

#include <ceres/ceres.h>

#include "multi_sensor_mapping/spline/ceres_spline_helper.h"
#include "multi_sensor_mapping/spline/ceres_spline_helper_jet.h"
#include "multi_sensor_mapping/spline/spline_segment.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

namespace auto_diff {

/**
 * @brief IMU全局速度因子
 *
 */
class IMUGlobalVelocityFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUGlobalVelocityFactor(const int64_t time_ns,
                          const Eigen::Vector3d& velocity,
                          const SplineMeta<SplineOrder>& spline_meta,
                          double vel_weight)
      : time_ns_(time_ns),
        velocity_(velocity),
        spline_meta_(spline_meta),
        vel_weight_(vel_weight) {
    inv_dt_ = 1e9 * 1.0 / spline_meta_.segments.begin()->dt_ns;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    size_t Knot_offset = spline_meta_.NumParameters();
    T time_offset_in_ns = sKnots[Knot_offset][0];
    T t_corrected = T(time_ns_) + time_offset_in_ns;

    size_t P_offset;
    T u;
    spline_meta_.ComputeSplineIndex(t_corrected, P_offset, u);

    Vec3T v_IkinG;
    CeresSplineHelperJet<T, SplineOrder>::template evaluate<3, 1>(
        sKnots + P_offset, u, inv_dt_, &v_IkinG);

    Eigen::Map<Vec3T> residuals(sResiduals);

    residuals = T(vel_weight_) * (v_IkinG - velocity_.template cast<T>());
    return true;
  }

  static auto* Create(const int64_t time_ns, const Eigen::Vector3d& velocity,
                      const SplineMeta<SplineOrder>& spline_meta,
                      double vel_weight) {
    using Functor = IMUGlobalVelocityFactor;
    return (new ceres::DynamicAutoDiffCostFunction<Functor>(
        new Functor(time_ns, velocity, spline_meta, vel_weight)));
  }

 private:
  int64_t time_ns_;
  Eigen::Vector3d velocity_;
  SplineMeta<SplineOrder> spline_meta_;
  double vel_weight_;
  double inv_dt_;
};

}  // namespace auto_diff

}  // namespace multi_sensor_mapping

#endif