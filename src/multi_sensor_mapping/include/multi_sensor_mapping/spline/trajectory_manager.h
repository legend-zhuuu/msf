#ifndef MSM_TRAJECTORY_MANAGER_H
#define MSM_TRAJECTORY_MANAGER_H

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_eigen.h"

namespace multi_sensor_mapping {

/**
 * @brief 时间参数
 *
 */
struct TimeParam {
  TimeParam() {
    traj_active = -1;
    for (int i = 0; i < 2; ++i) {
      lio_imu_time[i] = -1;
      lio_imu_idx[i] = 0;

      lio_map_imu_time[i] = -1;
      lio_map_imu_idx[i] = 0;

      last_scan[i] = -1;
      cur_scan[i] = -1;
    }
    last_bias_time = 0;
    cur_bias_time = 0;
  }

  /**
   * @brief 更新当前帧
   *
   * @param scan_time_min
   * @param scan_time_max
   */
  void UpdateCurScan(int64_t scan_time_min, int64_t scan_time_max) {
    last_scan[0] = cur_scan[0];
    last_scan[1] = cur_scan[1];

    cur_scan[0] = scan_time_min;
    cur_scan[1] = scan_time_max;
  }

  /// @brief 参与lio的IMU的时间区间
  int64_t lio_imu_time[2];
  /// @brief 参与lio的IMU的索引区间
  int lio_imu_idx[2];

  double lio_map_imu_time[2];
  int lio_map_imu_idx[2];

  /// @brief 此时刻之前的lio轨迹认为稳定
  double traj_active;
  /// @brief 上一帧激光的时间区间
  int64_t last_scan[2];
  /// @brief 当前帧激光的时间区间
  int64_t cur_scan[2];

  int64_t last_bias_time;
  int64_t cur_bias_time;
};

class Trajectory;
class ImuStateEstimator;
class MarginalizationInfo;
class OptimizationWeight;
class ClinsParams;

/**
 * @brief 轨迹管理器
 *
 */
class TrajectoryManager {
 public:
  typedef std::shared_ptr<TrajectoryManager> Ptr;

  /**
   * @brief Construct a new Trajectory Manager object 轨迹管理器
   *
   * @param _trajectory
   */
  TrajectoryManager(const std::shared_ptr<ClinsParams>& _clins_params,
                    const std::shared_ptr<Trajectory>& _trajectory);

  /**
   * @brief FeedImuData 喂入IMU数据
   *
   * @param _imu_data
   */
  void FeedImuData(const IMUData2& _imu_data);

  /**
   * @brief PropagateTrajectory 积分轨迹
   *
   * @param _scan_time_min
   * @param _scan_time_max
   */
  void PropagateTrajectory(int64_t _scan_time_min, int64_t _scan_time_max);

  /**
   * @brief PredictTrajectory 直接预测轨迹
   *
   * @param _scan_time_min
   * @param _scan_time_max
   */
  void PredictTrajectory(int64_t _scan_time_min, int64_t _scan_time_max);

  /**
   * @brief UndistortFeatures 矫正特征点
   *
   * @param _feature_data
   * @param _undistorted_cloud
   * @param _convert_to_map_frame_flag
   */
  void UndistortFeatures(const TimedCloudData2& _feature_data,
                         KRTPointCloudPtr& _undistorted_cloud,
                         bool _convert_to_map_frame_flag = false);

  /**
   * @brief Set the Original Pose object 设置初始位置
   *
   * @param _pos
   * @param _rot
   */
  void SetOriginalPose(const Eigen::Vector3d& _pos,
                       const Eigen::Quaterniond& _rot);

  /**
   * @brief Set the Initial State object 设置初始状态
   *
   * @param _imu_state
   */
  void SetInitialState(const IMUState& _imu_state);

 private:
  /**
   * @brief ExtendTrajectory 扩展轨迹
   *
   * @param _max_time_ns
   */
  void ExtendTrajectory(int64_t _max_time_ns);

  /**
   * @brief UpdateImuDataDuringLIO 更新LIO期间的IMU数据
   *
   */
  void UpdateImuDataDuringLIO();

  /**
   * @brief InitTrajectoryWithPropagation 基于积分结果初始化轨迹
   *
   */
  void InitTrajectoryWithPropagation(bool _use_velocity_factor = true);

  /**
   * @brief 时间戳是否在第一段中
   *
   * @param _time_cur
   * @return true
   * @return false
   */
  bool LocatedInFirstSegment(int64_t _time_cur);

 private:
  /// @brief 初始化位置
  PoseData2 original_pose_;

  /// @brief 轨迹
  std::shared_ptr<Trajectory> trajectory_;
  /// @brief IMU状态估计器
  std::shared_ptr<ImuStateEstimator> imu_state_estimator_;

  /// @brief IMU数据缓存
  Eigen::aligned_vector<IMUData2> imu_data_cache_;
  /// @brief 所有IMU的偏置
  std::map<int64_t, IMUBias> all_imu_bias_;

  /// @brief 时间参数
  TimeParam time_param_;
  /// @brief 优化参数
  std::shared_ptr<OptimizationWeight> optimization_weight_;

  /// @brief 重力
  Eigen::Vector3d gravity_;

  /// @brief 激光雷达先验信息
  std::shared_ptr<MarginalizationInfo> lidar_marg_info_;
  /// @brief 雷达先验控制点ID
  std::pair<int, int> lidar_prior_ctrl_id_;
  /// @brief 激光雷达参数块
  std::vector<double*> lidar_marg_parameter_blocks_;
};
}  // namespace multi_sensor_mapping

#endif