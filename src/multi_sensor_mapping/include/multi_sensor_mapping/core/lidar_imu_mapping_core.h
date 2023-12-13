#ifndef MSM_LIDAR_IMU_MAPPING_CORE_H
#define MSM_LIDAR_IMU_MAPPING_CORE_H

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class SensorParams;
class ExtrinsicParams;
class LidarImuMappingParams;
class FeatureExtractor;
class LOAMScanMatcher;
class ScanUndistortion;
class ImuPosePredictor;
class LidarMapSession;

/**
 * @brief The LidarImuMappingCore class
 * 激光-IMU建图核心
 */
class LidarImuMappingCore {
 public:
  typedef std::shared_ptr<LidarImuMappingCore> Ptr;

  /**
   * @brief LidarImuMapper 构造函数
   */
  LidarImuMappingCore();

  /**
   * @brief 析构函数
   *
   */
  ~LidarImuMappingCore();

  /**
   * @brief Set the Params object 设置参数
   *
   * @param _sensor_params
   * @param _extrinsic_params
   * @param _mapper_params
   */
  void SetParams(const std::shared_ptr<SensorParams>& _sensor_params,
                 const std::shared_ptr<ExtrinsicParams>& _extrinsic_params,
                 const std::shared_ptr<LidarImuMappingParams>& _mapper_params);

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
   * @param cloud_data
   * @param _imu_data_list
   */
  void OdometryOneScan(
      TimedCloudData& cloud_data,
      const std::vector<IMUData>& _imu_data_list = std::vector<IMUData>());

  /**
   * @brief SetInitializationStatus 设置初始化状态
   * @param start_time
   * @param init_rot
   */
  void SetInitializationStatus(double start_time, Eigen::Quaterniond init_rot);

  /**
   * @brief OnlyScanUndistortion 只进行点云去畸变(用于测试)
   * @param cloud_data
   */
  void OnlyScanUndistortion(
      TimedCloudData& cloud_data,
      const std::vector<IMUData>& _imu_data_list = std::vector<IMUData>());

  /**
   * @brief Get the Current Pose object 获取当前姿态
   *
   * @return Eigen::Matrix4d
   */
  PoseData GetCurrentPose();

  /**
   * @brief Get the Current Cloud object 获取当前点云
   *
   * @return CloudTypePtr
   */
  CloudTypePtr GetCurrentCloud();

  /**
   * @brief 是否为关键帧
   *
   * @return true
   * @return false
   */
  bool IsKeyFrame();

 private:
  /**
   * @brief InitImuPosePredictor 初始化姿态预测器
   */
  void InitImuPosePredictor();

  /**
   * @brief CheckKeyScan 检测关键帧
   * @param _scan_pose
   * @return
   */

  bool CheckKeyScan(const Eigen::Matrix4d& _scan_pose);

  /**
   * @brief InitPose 初始化姿态
   * @param imu_data_list
   */
  void InitPose(const std::vector<IMUData>& imu_data_list);

  /**
   * @brief DeskewScan 点云去畸变
   * @param _cloud_in
   * @param _imu_data_list
   */
  void DeskewScan(TimedCloudData& _cloud_in,
                  const std::vector<IMUData>& _imu_data_list);

  /**
   * @brief FeatureExtraction 特征提取
   */
  void FeatureExtraction();

  /**
   * @brief UpdatePrediction 更新预测姿态
   * @param _imu_data_list
   * @param _out_prediction
   */
  void UpdatePrediction(const std::vector<IMUData>& _imu_data_list,
                        Eigen::Matrix4d& _out_prediction);

  /**
   * @brief BuildSlidingWindowMap 构建滑窗点云
   */
  void BuildSlidingWindowMap();

  /**
   * @brief MatchScan 点云匹配
   * @param _prediction
   */
  bool MatchScan(const Eigen::Matrix4d& _prediction);

  /**
   * @brief UpdatePredictor 更新预测器
   */
  void UpdatePredictor();

  /**
   * @brief KeyframeSelection 关键帧选择
   */
  void KeyframeSelection();

  /**
   * @brief CheckAlignResult 检测预测结果
   * @param _prediction
   * @param _aligned
   * @return
   */
  bool CheckAlignResult(const Eigen::Matrix4d& _prediction,
                        const Eigen::Matrix4d& _aligned);

  /**
   * @brief CheckImuData IMU数据检测
   * @param imu_data_list
   * @return
   */
  void CheckImuData(const std::vector<IMUData>& imu_data_list);

 private:
  /// @brief 传感器参数
  std::shared_ptr<SensorParams> sensor_params_;
  /// @brief 外参参数
  std::shared_ptr<ExtrinsicParams> extrinsic_params_;
  /// @brief 建图参数
  std::shared_ptr<LidarImuMappingParams> mapper_params_;

  /// @brief 姿态预测器
  std::shared_ptr<ImuPosePredictor> pose_predictor_ptr_;
  /// @brief LOAM特征提取器
  std::shared_ptr<FeatureExtractor> feature_extractor_ptr_;
  /// @brief LOAM点云配准器
  std::shared_ptr<LOAMScanMatcher> scan_matcher_ptr_;
  /// @brief 点云矫正器
  std::shared_ptr<ScanUndistortion> scan_undistortion_ptr_;

  /// @brief 当前正在处理的建图任务
  std::shared_ptr<LidarMapSession> active_lidar_map_session_ptr_;

  /// @brief 参数初始化标志位
  bool param_initialized_flag_;
  /// @brief 第一帧标志位
  bool first_scan_flag_;

  /// @brief 最近的关键帧姿态的逆(用于判断是否新增关键帧)
  Eigen::Matrix4d latest_key_frame_pose_inv_;

  ///@brief 当前姿态
  Eigen::Matrix4d current_pose_;
  /// @brief 当前点云
  CloudTypePtr current_cloud_;
  /// @brief 当前去畸变后的原始rt点云
  VelRTPointCloudPtr current_deskewed_cloud_raw_;

  /// @brirf 当前点云时间戳
  double current_cloud_timestamp_;
  /// @brief 当前锚点信息
  int current_anchor_id_;
  /// @brief 当前特征点云
  CloudTypePtr current_corner_cloud_raw_;
  CloudTypePtr current_surface_cloud_raw_;

  /// @brief 关键帧更新的标志位
  bool keyframe_update_flag_;
  /// @brief 关键帧标志位
  bool keyframe_flag_;

  /// @brief 建图起始时间
  double map_start_time_;
  /// @brief 建图初始旋转(激光在全局坐标系下的旋转)
  Eigen::Quaterniond map_start_rot_;
};
}  // namespace multi_sensor_mapping
#endif
