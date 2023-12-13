#ifndef MSM_POSE_GRAPH_MANAGER_H
#define MSM_POSE_GRAPH_MANAGER_H

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <map>
#include <memory>

namespace multi_sensor_mapping {

class PoseGraphDatabase;
class LidarMapSession;

/**
 * @brief The PoseParams struct 姿态参数
 */
struct PoseParams {
  /// @brief 平移量(x,y,z)
  double t_param[3];
  /// @brief 旋转量(w,x,y,z)
  double q_param[4];
};

/**
 * @brief The PoseGraphManager class 基于ceres的后端优化
 */
class PoseGraphManager {
 public:
  typedef std::shared_ptr<PoseGraphManager> Ptr;

  /**
   * @brief PoseGraphManager 构造函数
   */
  PoseGraphManager();

  /**
   * @brief SetPoseGraphDatabase  设置共享位姿图数据库
   * @param _pgd_ptr
   */
  void SetPoseGraphDatabase(std::shared_ptr<PoseGraphDatabase> _pgd_ptr);

  /**
   * @brief SetExtGNSS2Lidar 设置GNSS到Lidar的外参
   * @param _ext_translation
   */
  void SetExtGNSS2Lidar(const Eigen::Vector3d& _ext_translation);

  /**
   * @brief SetExtGNSS2Lidar 设置GNSS到Lidar的外参
   * @param _ext_matrix
   */
  void SetExtGNSS2Lidar(const Eigen::Matrix4d& _ext_matrix);

  /**
   * @brief AddPoseNode 添加位姿节点
   * @param _id
   * @param _pose_t
   * @param _pose_q
   */
  void AddPoseNode(const int _id, const Eigen::Vector3d& _pose_t,
                   const Eigen::Quaterniond& _pose_q);

  /**
   * @brief AddPoseNode 添加位姿节点
   * @param _id
   * @param _pose
   */
  void AddPoseNode(const int _id, const Eigen::Matrix4d& _pose);

  /**
   * @brief SetPoseNodes 设置姿态节点序列
   * @param _pose_vec
   */
  void SetPoseNodes(const std::vector<Eigen::Matrix4d>& _pose_vec);

  /**
   * @brief AddLoopClosureEdge 添加闭环边
   * @param _front_id
   * @param _back_id
   * @param _delta_t
   * @param _delta_q
   * @param _score
   */
  void AddLoopClosureEdge(const int _front_id, const int _back_id,
                          const Eigen::Vector3d& _delta_t,
                          const Eigen::Quaterniond& _delta_q,
                          const double _score, bool _valid_flag = true);

  /**
   * @brief AddLoopClosureEdge 添加闭环边
   * @param _front_id
   * @param _back_id
   * @param _delta_pose
   * @param _score
   */
  void AddLoopClosureEdge(const int _front_id, const int _back_id,
                          const Eigen::Matrix4d& _delta_pose,
                          const double _score, bool _valid_flag = true);

  /**
   * @brief AddGpsEdge 添加GPS约束
   * @param _id
   * @param _gps_pose
   * @param _gps_noise
   * @param _use_elevation_flag
   */
  void AddGpsEdge(const int _id, const Eigen::Vector3d& _gps_pose,
                  const Eigen::Vector3d& _gps_noise,
                  bool _use_elevation_flag = true);

  /**
   * @brief UpdateLoopClosureEdge 更新闭环边
   *
   * @param _lc_id
   * @param _relative_pose
   * @param _score
   */
  void UpdateLoopClosureEdge(const int _lc_id,
                             const Eigen::Matrix4d& _relative_pose, int _score);

  /**
   * @brief BuildProblem 构建Pose graph 优化问题
   */
  void BuildProblem();

  /**
   * @brief SolveProblem 求解优化问题
   * @param _std_out
   * @param _max_iter
   */
  void SolveProblem(bool _std_out = true, int _max_iter = 50);

  /**
   * @brief 从map session导入地图信息
   *
   * @param _map_session
   */
  void ImportGraphFromMapSession(
      const std::shared_ptr<LidarMapSession>& _map_session);

  /**
   * @brief UpdateLidarMapSession 优化后更新姿态
   * @param _map_session
   */
  void UpdateLidarMapSession(std::shared_ptr<LidarMapSession>& _map_session);

  /**
   * @brief ClearLoopClosure
   */
  void ClearLoopClosure();

  /**
   * @brief 增量式闭环优化
   *
   */
  void IncrementalPoseGraphOptimization();

 public:
  /// @brief 帧间匹配协方差
  double scan_match_t_var_;
  double scan_match_q_var_;
  /// @brief 闭环协防差
  double loop_closure_t_var_;
  double loop_closure_q_var_;

  /// @brief 是否使用GPS边
  bool gps_edge_flag_;
  /// @brief GNSS到激光雷达的外参
  double p_GinL_[3];

  /// @brief 最新的闭环帧ID
  int latest_loop_closure_scan_id_;

  /// @brief 优化问题
  std::shared_ptr<ceres::Problem> problem_;
  /// @brief 顺序存放id
  std::vector<int> pose_id_vec_;
  /// @brief 姿态参数列表(不能使用std::vector)
  std::map<int, PoseParams> pose_params_map_;
  /// @brief 求解选项
  ceres::Solver::Options solve_options_;
  /// @brief 四元数的参数化
  ceres::LocalParameterization* local_parameterization_;

  /// @brief 位姿图数据库
  std::shared_ptr<PoseGraphDatabase> pose_graph_database_;
};

}  // namespace multi_sensor_mapping

#endif
