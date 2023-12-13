#ifndef MSM_TRAJECTORY_ESTIMATOR_H
#define MSM_TRAJECTORY_ESTIMATOR_H

#include <ceres/ceres.h>
#include <ceres/covariance.h>

#include "multi_sensor_mapping/factor/analytical_diff/marginalization_factor.h"
#include "multi_sensor_mapping/spline/ceres_local_param.h"
#include "multi_sensor_mapping/spline/spline_common.h"
#include "multi_sensor_mapping/spline/spline_segment.h"
#include "multi_sensor_mapping/spline/trajectory_estimator_options.h"
#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_common.h"

namespace multi_sensor_mapping {

class Trajectory;
class MarginalizationInfo;

/**
 * @brief 轨迹估计器
 *
 */
class TrajectoryEstimator {
  static ceres::Problem::Options DefaultProblemOptions() {
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }

 public:
  /**
   * @brief Construct a new Trajectory Estimator object 轨迹估计器构造函数
   *
   * @param _trajectory
   */
  TrajectoryEstimator(const std::shared_ptr<Trajectory>& _trajectory,
                      const TrajectoryEstimatorOptions& _option);

  /**
   * @brief Destroy the Trajectory Estimator object 轨迹估计器析构函数
   *
   */
  ~TrajectoryEstimator();

  /**
   * @brief Set the Key Scan Constant object 固定该帧之前的控制点
   *
   * @param _max_time
   */
  void SetKeyScanConstant(double _max_time);

  /**
   * @brief 时间戳转换判断测量时间是否在轨迹范围内
   *
   * @param _frame_type
   * @param _timestamp
   * @param _time_ns
   * @return true
   * @return false
   */
  bool MeasuredTimeToNs(const FrameType& _frame_type, const double& _timestamp,
                        int64_t& _time_ns);

  /**
   * @brief 判断测量时间是否在轨迹范围内
   *
   * @param _frame_type
   * @param _timestamp_ns
   * @return true
   * @return false
   */
  bool CheckMeasuredTime(const FrameType& _frame_type,
                         const int64_t& _timestamp_ns);

  /**
   * @brief Set the Fixed Index object 固定控制点索引
   *
   * @param _idx
   */
  void SetFixedIndex(int _idx);

  /**
   * @brief Get the Fixed Control Index object 获取固定控制点索引
   *
   * @return int
   */
  int GetFixedControlIndex();

  /**
   * @brief Set the Timeoffset State object 设置优化时间偏置状态
   *
   */
  void SetTimeoffsetState();

  /**
   * @brief 添加姿态测量
   *
   * @param _pose_data
   * @param _info_vec
   */
  void AddIMUPoseMeasurementAnalytic(
      const PoseData2& _pose_data,
      const Eigen::Matrix<double, 6, 1>& _info_vec);

  /**
   * @brief 添加IMU测量
   *
   * @param _imu_data
   * @param _gyro_bias
   * @param _accel_bias
   * @param _gravity
   * @param _info_vec
   * @param _marg_this_factor
   */
  void AddIMUMeasurementAnalytic(const IMUData2 _imu_data, double* _gyro_bias,
                                 double* _accel_bias, double* _gravity,
                                 const Eigen::Matrix<double, 6, 1>& _info_vec,
                                 bool _marg_this_factor = false);

  /**
   * @brief 添加激光测量
   *
   * @param _lidar_measurement
   * @param _S_GtoM
   * @param _p_GinM
   * @param _S_LtoI
   * @param _p_LinI
   * @param _weight
   * @param _marg_this_factor
   */
  void AddLidarMeasurementAnalytic(
      const PointCorrespondence& _lidar_measurement, const SO3d& _S_GtoM,
      const Eigen::Vector3d& _p_GinM, const SO3d& _S_LtoI,
      const Eigen::Vector3d& _p_LinI, double _weight,
      bool _marg_this_factor = false);

  /**
   * @brief 添加边缘化因子
   *
   * @param _last_marg_info
   * @param _last_marginalization_parameter_blocks
   */
  void AddMarginalizationFactor(
      const std::shared_ptr<MarginalizationInfo>& _last_marg_info,
      std::vector<double*>& _last_marginalization_parameter_blocks);

  /**
   * @brief 添加全局速度测量
   *
   * @param _time_ns
   * @param _velocity
   * @param _weight
   */
  void AddGlobalVelocityMeasurement(int64_t _time_ns,
                                    const Eigen::Vector3d& _velocity,
                                    double _weight);

  /**
   * @brief 边缘化做准备
   *
   * @param _r_type
   * @param _cost_function
   * @param _loss_function
   * @param _parameter_blocks
   * @param _drop_set
   */
  void PrepareMarginalizationInfo(ResidualType _r_type,
                                  ceres::CostFunction* _cost_function,
                                  ceres::LossFunction* _loss_function,
                                  std::vector<double*>& _parameter_blocks,
                                  std::vector<int>& _drop_set);

  /**
   * @brief 求解
   *
   * @param _max_iterations
   * @param _progress
   * @param _num_threads
   * @return ceres::Solver::Summary
   */
  ceres::Solver::Summary Solve(int _max_iterations = 50, bool _progress = false,
                               int _num_threads = -1);

 private:
  /**
   * @brief 添加控制点
   *
   * @param _spline_meta
   * @param _vec
   * @param _add_pos_knot
   */
  void AddControlPoints(const SplineMeta<SplineOrder>& _spline_meta,
                        std::vector<double*>& _vec, bool _add_pos_knot = false);

  /**
   * @brief 边缘化做准备
   *
   * @param r_type
   * @param spline_meta
   * @param cost_function
   * @param loss_function
   * @param parameter_blocks
   * @param drop_set_wo_ctrl_point
   */
  void PrepareMarginalizationInfo(ResidualType _r_type,
                                  const SplineMeta<SplineOrder>& _spline_meta,
                                  ceres::CostFunction* _cost_function,
                                  ceres::LossFunction* _loss_function,
                                  std::vector<double*>& _parameter_blocks,
                                  std::vector<int>& _drop_set_wo_ctrl_point);

 public:
  /// @brief 轨迹估计选项
  TrajectoryEstimatorOptions options_;

 private:
  /// @brief 轨迹
  std::shared_ptr<Trajectory> trajectory_;
  /// @brief 优化问题
  std::shared_ptr<ceres::Problem> problem_;
  /// @brief 参数化
  ceres::LocalParameterization* analytic_local_parameterization_;
  ceres::LocalParameterization* auto_diff_local_parameterization_;
  ceres::HomogeneousVectorParameterization* homo_vec_local_parameterization_;
  /// @brief 固定的控制点索引
  int fixed_control_point_index_;
  /// @brief 时间偏置参数
  std::map<FrameType, double*> time_offset_opt_params_;

  /// @brief 边缘化信息
  std::shared_ptr<MarginalizationInfo> marginalization_info_;
  /// @brief 回调函数
  std::vector<std::unique_ptr<ceres::IterationCallback>> callbacks_;
};
}  // namespace multi_sensor_mapping

#endif