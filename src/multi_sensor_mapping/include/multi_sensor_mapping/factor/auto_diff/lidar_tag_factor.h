#ifndef MSL_LIDAR_TAG_FACTOR_H
#define MSL_LIDAR_TAG_FACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Geometry>

namespace multi_sensor_mapping {

template <typename T>
inline void InspiredCost(const T& lambda, const T& a, T& c) {
  if (lambda > a) {
    c = lambda - a;
  } else if (lambda < -a) {
    c = -lambda - a;
  } else {
    c = T(0.0);
  }
}

class LidarTagFactor {
 public:
  LidarTagFactor(const Eigen::Vector3d& _tag_point, const double& _tag_bound)
      : tag_point_(_tag_point), tag_bound_(_tag_bound) {}

  /**
   * @brief 构建残差
   *
   * @param transform position:(x,y,z) rotation:(x,y,z,w)
   * @param residuals 残差：变换后与已知点的距离
   * @return true

   */
  template <class T>
  bool operator()(const T* const transform, T* residuals) const {
    // T_tag2lidar
    T t_tag2lidar[3];
    t_tag2lidar[0] = T(transform[0]);
    t_tag2lidar[1] = T(transform[1]);
    t_tag2lidar[2] = T(transform[2]);

    // R_tag2lidar
    T r_tag2lidar[4];
    r_tag2lidar[0] = T(transform[3]);
    r_tag2lidar[1] = T(transform[4]);
    r_tag2lidar[2] = T(transform[5]);
    r_tag2lidar[3] = T(transform[6]);

    // P inside tag
    T tag_p[3];
    tag_p[0] = T(tag_point_[0]);
    tag_p[1] = T(tag_point_[1]);
    tag_p[2] = T(tag_point_[2]);

    // RP
    T origin_p_trans[3];
    ceres::QuaternionRotatePoint(r_tag2lidar, tag_p, origin_p_trans);

    // RP+T
    origin_p_trans[0] += t_tag2lidar[0];
    origin_p_trans[1] += t_tag2lidar[1];
    origin_p_trans[2] += t_tag2lidar[2];

    // 转换后的顶点位置应与虚拟tag板顶点一致
    // delta_x delta_y delta_z
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);

    T inspired_cost[3];
    InspiredCost(origin_p_trans[0], T(0.02), inspired_cost[0]);
    InspiredCost(origin_p_trans[1], T(tag_bound_), inspired_cost[1]);
    InspiredCost(origin_p_trans[2], T(tag_bound_), inspired_cost[2]);

    res[0] = inspired_cost[0];
    res[1] = inspired_cost[1];
    res[2] = inspired_cost[2];

    return true;
  }

  /**
   * @note input(7): position:(x,y,z),rotation(x,y,z,w)
   * residual(3): delta_x,delta_y,delta_z
   *
   * @return ceres::CostFunction*
   */
  static ceres::CostFunction* Create(const Eigen::Vector3d& _tag_point,
                                     const double& _tag_bound) {
    return (new ceres::AutoDiffCostFunction<LidarTagFactor, 3, 7>(
        new LidarTagFactor(_tag_point, _tag_bound)));
  }

 private:
  /// @brief tag板内点 在 liar坐标系下的位置
  Eigen::Vector3d tag_point_;
  /// @brief 坐标变换后点位置约束
  double tag_bound_;
};
}  // namespace multi_sensor_mapping

#endif