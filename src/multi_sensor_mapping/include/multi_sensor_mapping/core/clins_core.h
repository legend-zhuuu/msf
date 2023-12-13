#ifndef MSM_CLINS_CORE_H
#define MSM_CLINS_CORE_H

#include <memory>

#include "ikd-Tree/ikd_Tree.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class SensorParams;
class ExtrinsicParams;
class ClinsParams;
class LidarMapSession;
class Trajectory;
class LidarMapSession;
class TrajectoryManager;

typedef std::vector<KRTPoint, Eigen::aligned_allocator<KRTPoint>> KPointVector;

/**
 * @brief 基于连续时间轨迹的激光-IMU融合建图核心
 *
 */
class ClinsCore {
 public:
  typedef std::shared_ptr<ClinsCore> Ptr;

  /**
   * @brief Construct a new Clins Core object 构造函数
   *
   */
  ClinsCore();

  /**
   * @brief Set the Params object 设置参数
   *
   * @param _sensor_params
   * @param _extrinsic_params
   * @param _mapper_params
   */
  void SetParams(const std::shared_ptr<SensorParams>& _sensor_params,
                 const std::shared_ptr<ExtrinsicParams>& _extrinsic_params,
                 const std::shared_ptr<ClinsParams>& _mapper_params);

  /**
   * @brief Set the Lidar Map Session object 设置激光地图任务
   *
   * @param _map_session
   */
  void SetLidarMapSession(const std::shared_ptr<LidarMapSession>& _map_session);

  /**
   * @brief InitializeAll 初始化
   */
  void InitializeAll();

  /**
   * @brief ProcessOneScan 里程计处理一帧Lidar数据
   * @param _cloud_data
   * @param _imu_data_list
   */
  void OdometryOneScan(
      TimedCloudData2& _cloud_data,
      const std::vector<IMUData2>& _imu_data_list = std::vector<IMUData2>());

  /**
   * @brief Set the Initial State object 设置初始状态
   *
   * @param _initial_state
   * @param _imu_data
   */
  void SetInitialState(const IMUState& _initial_state,
                       const IMUData2& _imu_data);

 private:
  /**
   * @brief 输入IMU数据
   *
   * @param _imu_data_list
   * @return true
   * @return false
   */
  void FeedImuDataVec(const std::vector<IMUData2>& _imu_data_list);

  /**
   * @brief 点云数据预处理
   *
   * @param _cloud_data
   */
  void PreprocessCloudData(const TimedCloudData2& _cloud_data);

  /**
   * @brief 更新关键帧
   *
   */
  bool UpdateKeyFrame();

  /**
   * @brief 更新局部地图
   *
   * @return true
   * @return false
   */
  void UpdateLocalFeatureMap();

  /**
   * @brief 检测是否为关键帧
   *
   * @param _timestamp_ns
   * @return true
   * @return false
   */
  bool CheckKeyFrame(int64_t _timestamp_ns);

  /**
   * @brief 特征关联
   *
   * @param _feature_in_global
   * @return int
   */
  int FeatureAssociation(const KRTPointCloudPtr& _feature_in_global);

  /**
   * @brief 基于PCA的平面判断
   *
   * @param _points
   * @param _threshold
   * @param _plane_coeff
   * @return true
   * @return false
   */
  bool EstimatePlane(const KPointVector& _points, const float& _threshold,
                     Eigen::Vector4f& _plane_coeff);

 private:
  /// @brief 传感器参数
  std::shared_ptr<SensorParams> sensor_params_;
  /// @brief 外参参数
  std::shared_ptr<ExtrinsicParams> extrinsic_params_;
  /// @brief 建图参数
  std::shared_ptr<ClinsParams> mapper_params_;

  /// @brief 轨迹
  std::shared_ptr<Trajectory> trajectory_;
  /// @brief 轨迹管理器
  std::shared_ptr<TrajectoryManager> trajectory_manager_;
  /// @brief 当前正在处理的建图任务
  std::shared_ptr<LidarMapSession> active_lidar_map_session_;

  /// @brief 参数初始化标志位
  bool param_initialized_flag_;
  /// @brief 添加关键帧的时间阈值
  int64_t keyframe_adding_time_ns_threshold_;

  /// @brief 关键帧更新的标志位
  bool keyframe_update_flag_;
  /// @brief 关键帧标志位
  bool keyframe_flag_;

  /// @brief 当前处理的特征点云
  TimedCloudData2 current_feature_data_;

  /// @brief 缓存的特征点云
  std::map<int64_t, TimedCloudData2> cache_feature_container_;
  /// @brief 局部特征点云
  std::vector<TimedCloudData2> local_feature_container_;

  /// @brief 上一帧关键帧时间戳
  int64_t last_keyframe_timestamp_ns_;
  /// @brief 上一帧关键帧位姿的逆
  SE3d last_keyframe_pose_inv_;

  /// @brief 初始状态
  IMUState initial_state_;
  /// @brief 增量式kd-tree
  KD_TREE<KRTPoint> ikd_tree_;
  /// @brief 最近点
  std::vector<KPointVector> nearest_points_;
  /// @brief 特征点是否被选中
  std::vector<bool> feature_selected_flag_container_;
  /// @brief 特征关联
  std::vector<PointCorrespondence> feature_corr_container_;
};
}  // namespace multi_sensor_mapping

#endif