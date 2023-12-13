#ifndef MSM_SDK_TYPES_H
#define MSM_SDK_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <vector>

namespace msm_sdk {

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef CloudType::Ptr CloudTypePtr;

/**
 * @brief 版本号
 *
 */
struct Version {
  Version(int _marjor = 0, int _minor = 0, int _patch = 0);

  std::string toString();

  int major_ = 0;  //!< Major number of the version
  int minor_ = 0;  //!< Minor number of the version
  int patch_ = 0;  //!< Patch number of the version
};

/**
 * @brief IMU数据定义
 *
 */
struct IMU {
  /// @brief 时间戳
  double timestamp = 0.0;
  /// @brief 角速度
  Eigen::Vector3d gyro = Eigen::Vector3d(0, 0, 0);
  /// @brief 加速度
  Eigen::Vector3d accel = Eigen::Vector3d(0, 0, 0);
  /// @brief 方向
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 姿态
 *
 */
struct StampedPose {
  /// @brief 时间戳
  double timestamp = 0;
  /// @brief 位置
  Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
  /// @brief 方向
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 点云
 *
 */
struct StampedCloud {
  /// @brief 时间戳
  double timestamp = 0;
  /// @brief 点云
  CloudType cloud;
};

/**
 * @brief 建图场景
 *
 */
enum MappingScene {
  SCENE_DAFAULT = 0,  // 默认场景
  SCENE_INDOOR,       // 室内
  SCENE_OUTDOOR       // 室外
};

/**
 * @brief 闭环边
 *
 */
struct LoopClosureEdge {
  /// @brief 闭环ID号
  int id;
  /// @brief ID号
  int front_id;
  int back_id;
  /// @brief 配准得分
  double reg_score;
  /// @brief 是否有效
  bool valid_flag;
};

/**
 * @brief 位姿图
 *
 */
struct Graph {
  /// @brief 顶点
  std::vector<Eigen::Vector3d> vertices;
  /// @brief 闭环边
  std::vector<LoopClosureEdge> loop_closure_edges;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 闭环信息
 *
 */
struct LoopClosureInfo {
  /// @brief 闭环ID
  int loop_closure_id;
  /// @brief 目标点云
  CloudType target_cloud;
  /// @brief 源点云
  CloudType source_cloud;
  /// @brief 目标点云位置
  Eigen::Vector3d target_pos;
  /// @brief 源点云位置
  Eigen::Vector3d source_pos;
  /// @brief 相对位置(x, y, z)
  Eigen::Vector3d relative_pos;
  /// @brief 相对姿态欧拉角(roll, pitch, yaw)
  Eigen::Vector3d relative_euler;

  /// @brief 分数
  double score;
};

}  // namespace msm_sdk

#endif