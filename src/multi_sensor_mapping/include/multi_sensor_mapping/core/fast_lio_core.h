#ifndef MSM_FAST_LIO_CORE_H
#define MSM_FAST_LIO_CORE_H

#include <Eigen/Eigen>
#include <memory>

#include "ikd-Tree/ikd_Tree.h"
#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/use_ikfom.h"

#define LASER_POINT_COV (0.001)
#define NUM_MATCH_POINTS (5)

typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

namespace multi_sensor_mapping {

class ExtrinsicParams;
class FastLIOParams;
class ImuProcessor;

/**
 * @brief FAST-LIO核心
 *
 */
class FastLIOCore {
 public:
  /**
   * @brief Construct a new Fast LIO Core object
   *
   */
  FastLIOCore();

  /**
   * @brief Set the Params object
   *
   * @param _extrinsic_params
   * @param _lio_params
   */
  void SetParams(const std::shared_ptr<ExtrinsicParams>& _extrinsic_params,
                 const std::shared_ptr<FastLIOParams>& _lio_params);

  /**
   * @brief InitializeAll 初始化
   *
   */
  void InitializeAll();

  /**
   * @brief OdometryOnceScan 里程推算一帧数据
   *
   * @param _cloud_data
   * @param _imu_data_list
   */
  void OdometryOnceScan(const TimedCloudData& _cloud_data,
                        const std::deque<IMUData>& _imu_data_list);

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

 private:
  /**
   * @brief ObsModel 激光观测模型
   *
   * @param s
   * @param ekfom_data
   */
  void ObsModel(state_ikfom& s,
                esekfom::dyn_share_datastruct<double>& ekfom_data);

  /**
   * @brief LaserMapFovSegment 动态调整局部地图
   *
   */
  void LaserMapFovSegment();

  /**
   * @brief PointBodyToWorld 转换到全局坐标系
   *
   * @param _pi
   * @param _po
   */
  void PointBodyToWorld(PointType const* const _pi, PointType* const _po);

  /**
   * @brief 地图增量更新
   *
   */
  void MapIcremental();

  /**
   * @brief 估计平面
   *
   * @param _pca_result
   * @param _point
   * @param _threshold
   * @return true
   * @return false
   */
  bool EstimatePlane(Eigen::Vector4f& _pca_result, const PointVector& _point,
                     const float& _threshold);

 private:
  /// @brief 地图移动阈值
  const float MOV_THRESHOLD = 1.5f;

  /// @brief 外参参数
  std::shared_ptr<ExtrinsicParams> extrinsic_params_;
  /// @brief FAST-LIO参数
  std::shared_ptr<FastLIOParams> lio_params_;

  /// @brief 激光-IMU平移外参
  Eigen::Vector3d lidar_to_imu_translation_;
  /// @brief 激光-IMU旋转外参
  Eigen::Quaterniond lidar_to_imu_rotation_;

  /// @brief IMU处理
  std::shared_ptr<ImuProcessor> imu_processor_;
  /// @brief  IKD tree
  KD_TREE<PointType> ikd_tree_;
  /// @brief 滤波器
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;

  /// @brief 局部地图是否初始化标志位
  bool local_map_initialized_;
  /// @brief EKF是否初始化标志位
  bool efk_initialized_flag_;

  double cube_length_;
  /// @brief 删除范围
  double delete_range_;

  /// @brief 矫正后的特征点
  CloudTypePtr feature_undistort_;
  /// @brief 矫正后的降采样特征点
  CloudTypePtr feature_undistort_downsampled_body_;
  /// @brief  矫正后的降采样特征点
  CloudTypePtr feature_undistort_downsampled_world_;
  /// @brief 特征关联信息
  CloudTypePtr norm_vec_;
  /// @brief 原始点云数据
  CloudTypePtr laser_cloud_origin_;
  /// @brief 关联后的平面参数
  CloudTypePtr corr_norm_vec_;

  /// @brief 降采样后特征点数量
  int feature_ds_size_;

  /// @brief 需要被删除的cube
  std::vector<BoxPointType> cube_need_rm_;
  /// @brief 局部地图
  BoxPointType local_map_points_;
  /// @brief 临近点集合
  std::vector<PointVector> nearest_points_;
  /// @brief kd-tree删除计数器
  int kdtree_delete_counter_;
  /// @brief 面元被选择标志位
  std::vector<bool> point_selected_surf_flag_;
  /// @brief 残差
  std::vector<float> res_last_;

  /// @brief 最新的IMU状态
  state_ikfom latest_state_;
  /// @brief 当前点云时间戳
  double current_cloud_timestamp_;
  /// @brief 激光雷达姿态
  Eigen::Vector3d lidar_position_;
  /// @brief 激光雷达旋转
  Eigen::Quaterniond lidar_rotation_;

  /// @brief 特征点滤波器
  pcl::VoxelGrid<PointType> feature_voxel_filter_;
};

}  // namespace multi_sensor_mapping

#endif