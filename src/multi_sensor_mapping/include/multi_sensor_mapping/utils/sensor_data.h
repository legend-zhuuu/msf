#ifndef MSM_SENSOR_DATA_H
#define MSM_SENSOR_DATA_H

#include <Eigen/Eigen>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "multi_sensor_mapping/utils/utils_sophus.h"

namespace multi_sensor_mapping {

/**
 * @brief The IMUData struct IMU的数据
 */
struct IMUData {
  /// @brief 时间戳
  double timestamp = 0.0;
  /// @brief 与上一帧的时间差
  double dt = 0.0;
  /// @brief 角速度
  Eigen::Vector3d gyro = Eigen::Vector3d(0, 0, 0);
  /// @brief 加速度
  Eigen::Vector3d accel = Eigen::Vector3d(0, 0, 0);
  /// @brief 方向
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  /// @brief 轮速计信息 (odom_vel[2] == 0 无效数据)
  Eigen::Vector3d odom_vel = Eigen::Vector3d(0, 0, 0);
  /// @brief 静止标志位(根据轮速计速度判断)
  bool stationary_flag = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief IMU数据
 *
 */
struct IMUData2 {
  /// @brief 时间戳
  int64_t timestamp_ns;
  /// @brief 角速度
  Eigen::Vector3d gyro;
  /// @brief 加速度
  Eigen::Vector3d accel;
  /// @brief 方向
  SO3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief IMU偏置
 *
 */
struct IMUBias {
  IMUBias()
      : gyro_bias(Eigen::Vector3d::Zero()),
        accel_bias(Eigen::Vector3d::Zero()) {}
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d accel_bias;
};

/**
 * @brief IMU状态
 *
 */
struct IMUState {
  IMUState()
      : timestamp_ns(0),
        position(Eigen::Vector3d::Zero()),
        velocity(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        gravity(Eigen::Vector3d(0, 0, -9.8)) {}
  /// @brief  时间戳
  int64_t timestamp_ns;
  /// @brief 位置
  Eigen::Vector3d position;
  /// @brief 速度
  Eigen::Vector3d velocity;
  /// @brief 方向
  Eigen::Quaterniond orientation;
  /// @brief 偏置
  IMUBias bias;
  /// @brief 重力
  Eigen::Vector3d gravity;
};

/**
 * @brief 外参参数
 *
 */
struct ExtParam {
  ExtParam()
      : p(Eigen::Vector3d::Zero()),
        q(Eigen::Quaterniond::Identity()),
        t_offset_ns(0) {}

  void UpdateParam() {
    so3 = SO3d(q);
    se3 = SE3d(so3, p);
  }

  Eigen::Vector3d p;
  SO3d so3;              /// ceres 优化时参数地址
  Eigen::Quaterniond q;  /// ceres 优化时参数地址
  SE3d se3;
  double t_offset_ns;
};

/**
 * @brief The PoseData struct 三维姿态数据
 */
struct PoseData {
  /// @brief 时间戳
  double timestamp = 0;
  /// @brief 位置
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// @brief 方向
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief The PoseData2 struct 三维姿态数据
 *
 */
struct PoseData2 {
  /// @brief 时间戳
  int64_t timestamp_ns = 0;
  /// @brief 位置
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// @brief 方向
  SO3d orientation = SO3d(Eigen::Quaterniond::Identity());

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief The AnchorData struct 锚点信息
 */
struct AnchorData {
  /// @brief 锚点ID号
  int anchor_id;
  /// @brief 锚点时间戳
  double timestamp;
  /// @brief 锚点位置(RTK)
  Eigen::Vector3d anchor_position;
};

/**
 * @brief The RelativePoseData struct 相对位姿
 */
struct RelativePoseData {
  double ref_time;
  double cur_time;
  Eigen::Vector3d relative_pos;
  Eigen::Quaterniond relative_rot;

  void ComputeRelativePose(const PoseData& ref_pose, const PoseData& cur_pose) {
    ref_time = ref_pose.timestamp;
    cur_time = cur_pose.timestamp;

    relative_rot = ref_pose.orientation.inverse() * cur_pose.orientation;
    relative_pos = ref_pose.orientation.inverse() *
                   (cur_pose.position - ref_pose.position);
  }
};

/**
 * @brief The TimedCloudData struct 一帧点云数据(统一使用velodyne格式)
 */
struct TimedCloudData {
  /// @brief 时间戳
  double timestamp = 0;
  /// @brief 点云数据
  VelRTPointCloudPtr cloud;
  /// @brief 锚点ID
  int anchor_id = -1;
  /// @brief 有效标志位
  bool valid_flag = false;
};

struct TimedCloudData2 {
  /// @brief 帧起始时间戳
  int64_t scan_start_timestamp_ns;
  /// @brief 帧结束时间戳
  int64_t scan_end_timestamp_ns;
  /// @brief 点云数据
  KRTPointCloudPtr cloud;
};

/**
 * @brief The OdometryData struct 里程计数据
 */
struct OdometryData {
  /// @brief 时间戳
  double timestamp;
  /// @brief
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  Eigen::Matrix4d Matrix() {
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    res.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    res.block<3, 1>(0, 3) = position;
    return res;
  }
};

/**
 * @brief The OptFramePose struct 关键帧姿态优化变量
 */
struct OptFramePose {
  double timestamp;
  int frame_id;
  /// @brief  优化变量
  Eigen::Vector3d opt_position;
  Eigen::Quaterniond opt_orientation;

  /// @brief 锚点姿态
  Eigen::Vector3d anchor_position;
  Eigen::Quaterniond anchor_orientation;
};

/**
 * @brief The OptMapPoint struct 地图点优化变量
 */
struct OptMapPoint {
  int id;

  bool bad_flag = false;

  /// @brief 世界坐标系姿态
  Eigen::Vector3d world_pos;
  /// @brief 观测
  std::vector<std::pair<int, Eigen::Vector2d>> observations;

  std::vector<float> sigma_squares;

  std::vector<bool> opt_flags;
};

/**
 * @brief The Reflector struct 反光柱
 */
struct ReflectionColumn {
  /// @brief 反光柱ID号
  int id;
  /// @brief 反光柱姿态
  Eigen::Vector3d position;
  /// @brief 关键帧的观测
  std::vector<std::pair<int, Eigen::Vector3d>> observations;
  /// @brief 反光柱直径
  double diamater;
  /// @brief 反光柱长度
  double length;
};

/**
 * @brief The GNSSData struct GNSS数据
 */
struct GNSSData {
  /// @brief 时间戳
  double timestamp = 0.0;
  /// @brief 坐标(lat, lon, alt)
  Eigen::Vector3d lla = Eigen::Vector3d(0, 0, 0);
  /// @brief 局部ENU坐标
  Eigen::Vector3d lenu = Eigen::Vector3d(0, 0, 0);
  /// @brief 标准差
  Eigen::Vector3d std = Eigen::Vector3d(0, 0, 0);

  /// @brief 航向角(deg)
  double yaw = 0.0;
  /// @brief 航向角的标准差
  double yaw_deviation = -1;

  /// @brief 状态 (-1 : 未fix )
  int status = 0;
};

/**
 * @brief 关联点
 *
 */
struct PointCorrespondence {
  /// @brief 点时间戳
  int64_t point_timestamp_ns;
  /// @brief 地图时间戳
  int64_t map_timestamp_ns;
  /// @brief 原始测量点
  Eigen::Vector3d point;

  /// @brief 平面参数
  Eigen::Vector4d geo_plane;
  /// @brief 距离参数
  double distance_to_plane;
};

}  // namespace multi_sensor_mapping

#endif
