#ifndef MSM_AUTO_DIFF_COLUMN_FACTOR_H
#define MSM_AUTO_DIFF_COLUMN_FACTOR_H

#include <ceres/ceres.h>

#include <Eigen/Eigen>

namespace multi_sensor_mapping {

/**
 * @brief 反光柱因子
 *
 */
class ColumnFactor {
 public:
  ColumnFactor(const Eigen::Vector3d& _column_obs) : colunm_obs_(_column_obs) {}

  template <class T>
  bool operator()(const T* const param, T* residuals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    // pose: base pose in map frame
    Eigen::Matrix<T, 2, 1> translation(param[0][0], param[0][1]);
    Eigen::Rotation2D<T> rotation(param[0][2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform_base2map;
    transform_base2map << rotation_matrix, translation, T(0.), T(0.), T(1.);

    Eigen::Map<Vec3T const> const column_position(param[1]);

    Eigen::Matrix<T, 3, 1> column_obs(T(colunm_obs_(0)), T(colunm_obs_(1)),
                                      T(colunm_obs_(2)));
    // map坐标系下反光柱观测位置
    Eigen::Matrix<T, 3, 1> map2column_obs = transform_base2map * column_obs;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = map2column_obs - column_position;

    return true;
  }

 private:
  /// @brief 反光柱观测
  Eigen::Vector3d colunm_obs_;
};
}  // namespace multi_sensor_mapping

#endif
