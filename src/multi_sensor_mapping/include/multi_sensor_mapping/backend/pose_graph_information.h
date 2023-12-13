#ifndef MSM_POSE_GRAPH_INFORMATION_H
#define MSM_POSE_GRAPH_INFORMATION_H

#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief The GNSSConstraint struct  GNSS约束
 */
struct GNSSConstraint {
  /// @brief 对应的ID
  int id;
  /// @brief gnss姿态
  Eigen::Vector3d gnss_pose;
  /// @brief gnss噪声
  Eigen::Vector3d gnss_noise;
  /// @brief 是否使用gnss高度
  bool use_gps_elevation;
  /// @brief 是否有效
  bool valid_flag;
};

/**
 * @brief The LoopClosureConstraint struct 闭环约束
 */
struct LoopClosureConstraint {
  /// @brief ID号
  int front_id;
  /// @brief ID号
  int back_id;

  /// @brief 位置差
  Eigen::Vector3d delta_position;
  /// @brief 旋转差
  Eigen::Quaterniond delta_rotation;
  /// @brief 配准得分
  double reg_score;
  /// @brief 是否有效
  bool valid_flag;
};

/**
 * @brief 顶点信息
 *
 */
struct GraphVertex {
  /// @brief ID号
  int id;
  /// @brief 位置
  Eigen::Vector3d position;
  /// @brief 方向
  Eigen::Quaterniond orientation;
  /// @brief 是否有效
  bool valid_flag;
};

/**
 * @brief 位姿图
 *
 */
struct PoseGraph {
  /// @brief 顶点
  std::vector<GraphVertex> vertices;
  /// @brief GNSS边
  std::vector<GNSSConstraint> gnss_edges;
  /// @brief 闭环边
  std::vector<LoopClosureConstraint> loop_closure_edegs_;
};

/**
 * @brief 局部匹配信息
 *
 */
struct LocalMatchInformation {
  /// @brief 闭环ID
  int loop_closure_id;
  /// @brief 目标点云
  CloudType target_cloud;
  /// @brief 源点云
  CloudType source_cloud;
  /// @brief 目标点云在地图坐标系的位置
  Eigen::Matrix4d target_pose_in_map;
  /// @brief 源点云在地图坐标系的位置
  Eigen::Matrix4d source_pose_in_map;
  /// @brief 相对姿态
  Eigen::Matrix4d relative_pose;
  /// @brief 分数
  double score;
};

/**
 * @brief The PoseGraphInformation class 位姿图信息数据库
 */
class PoseGraphDatabase {
 public:
  typedef std::shared_ptr<PoseGraphDatabase> Ptr;

  /**
   * @brief PoseGraphDatabase 构造函数
   */
  PoseGraphDatabase();

  /**
   * @brief AddLoopClosureConstraint 增加闭环约束
   * @param _loop_closure_constraint
   */
  void AddLoopClosureConstraint(
      LoopClosureConstraint& _loop_closure_constraint);

  /**
   * @brief AddGnssConstraint 增加GNSS约束
   * @param _gnss_constraint
   */
  void AddGnssConstraint(GNSSConstraint& _gnss_constraint);

  /**
   * @brief Save 保存
   * @param _cache_path
   * @return
   */
  bool Save(std::string _cache_path);

  /**
   * @brief Load 加载
   * @param _cache_path
   * @return
   */
  bool Load(std::string _cache_path);

  /**
   * @brief ClearLoopClosureConstraint 清空闭环约束
   */
  void ClearLoopClosureConstraint();

  /**
   * @brief ClearGnssCOnstraint 清空GNSS约束
   */
  void ClearGnssCOnstraint();

  /**
   * @brief LoopClosureConstraintEmpty 闭环约束是否为空
   * @return
   */
  inline bool LoopClosureConstraintEmpty() {
    return loop_closure_constraint_info_.empty();
  }

  /**
   * @brief GnssConstraintEmpty GNSS约束是否为空
   * @return
   */
  inline bool GnssConstraintEmpty() { return gnss_constraint_info_.empty(); }

 public:
  /// @brief 闭环约束ID号集合
  std::vector<int> loop_closure_info_id_vec_;
  /// @brief 闭环约束信息
  std::unordered_map<int, LoopClosureConstraint> loop_closure_constraint_info_;
  /// @brief GNSS约束ID号集合
  std::vector<int> gnss_constraint_id_vec_;
  /// @brief GNSS约束信息
  std::unordered_map<int, GNSSConstraint> gnss_constraint_info_;
  /// @brief 闭环边ID号
  int loop_closure_constraint_id_;
  /// @brief GNSS约束ID号
  int gnss_constraint_id_;
};

}  // namespace multi_sensor_mapping

#endif
