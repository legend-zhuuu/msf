#ifndef MSM_LIDAR_MAP_UNIT_H
#define MSM_LIDAR_MAP_UNIT_H

#include <Eigen/Eigen>
#include <memory>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief The LidarMapUnit class 激光地图最小单元（3D）
 */
class LidarMapUnit {
 public:
  typedef std::shared_ptr<LidarMapUnit> Ptr;

  /**
   * @brief LidarMapUnit 构造函数
   */
  LidarMapUnit();

  /**
   * @brief LidarMapUnit 构造函数
   * @param _timestamp
   * @param _full_cloud
   * @param _corner_cloud
   * @param _surface_cloud
   * @param _pose
   */
  LidarMapUnit(double _timestamp, CloudTypePtr _full_cloud,
               CloudTypePtr _corner_cloud, CloudTypePtr _surface_cloud,
               Eigen::Matrix4d& _pose);

  /**
   * @brief LidarMapUnit 构造函数
   * @param _timestamp
   * @param _full_cloud
   * @param _corner_cloud
   * @param _surface_cloud
   * @param _pose
   * @param _ground_coeff
   */
  LidarMapUnit(double _timestamp, CloudTypePtr _full_cloud,
               CloudTypePtr _corner_cloud, CloudTypePtr _surface_cloud,
               Eigen::Matrix4d& _pose, Eigen::Vector4d& _ground_coeff);

  /**
   * @brief LidarMapUnit 构造函数
   * @param _timestamp
   * @param _full_cloud
   * @param _pose
   */
  LidarMapUnit(double _timestamp, CloudTypePtr _full_cloud,
               Eigen::Matrix4d& _pose);

  /**
   * @brief GetFullCloud
   * @return
   */
  inline CloudTypePtr GetFullCloud() { return full_cloud_; }

  /**
   * @brief GetPose
   * @return
   */
  Eigen::Matrix4d GetPose();

 public:
  /// @brief 下一个ID号
  static int next_unit_id;

  /// @brief ID号
  int unit_id_;
  /// @brief 时间戳
  double timestamp_;

  /// @breif 全部点云(去畸变)
  CloudTypePtr full_cloud_;
  /// @brief 角点特征点云(降采样)
  CloudTypePtr corner_feature_cloud_ds_;
  /// @brief 平面特征点云(降采样)
  CloudTypePtr surface_feature_cloud_ds_;

  /// @brief 局部地图坐标系位置
  Eigen::Vector3d position_;
  /// @brief 局部地图坐标系旋转
  Eigen::Quaterniond orientation_;
  /// @brief local ENU坐标系下的位置
  Eigen::Vector3d LENU_position_;
  /// @brief local ENU坐标系下的旋转
  Eigen::Quaterniond LENU_orientation_;

  /// @brief 是否可用标志位
  bool bad_flag_;
  /// @brief 地面参数
  Eigen::Vector4d ground_coeff_;

  /// @brief 切片属性(切片半径，初始高度，切片厚度) 重力对齐之后
  Eigen::Vector3d slice_attribute_;

  /// @brief 锚点标志位
  int anchor_point_id_;
};

}  // namespace multi_sensor_mapping
#endif
