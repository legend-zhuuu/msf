#ifndef MSM_LOOP_CLOSURE_DETECTOR_H
#define MSM_LOOP_CLOSURE_DETECTOR_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

#define CRS_ICP 1
#define CRS_NDT 2
#define CRS_NDT_ICP 3
#define CRS_GICP 4
#define CLOUD_REVALIDATION_STRATEGY CRS_GICP

namespace multi_sensor_mapping {

class LidarMapSession;

struct LoopClosureResult {
  ///@brief 检测结果,true为成功,false为失败
  bool success_flag = false;
  /// @brief 当前帧ID
  int current_scan_id_ = -1;
  /// @brief 历史帧ID(<0 则没找到闭环)
  int history_scan_id_ = -1;
  /// @brief 相对变化矩阵(history_scan 为基准)
  Eigen::Matrix4d trans_history_to_current = Eigen::Matrix4d::Identity();
  /// @brief 分数
  double score_ = 0;
};

/**
 * @brief 闭环候选
 *
 */
struct LoopCandidate {
  /// @brief  候选历史帧
  int key_history;
  /// @brief 候选当前帧
  int key_cur;
  /// @brief 欧式距离
  double eucliden_distance;
  /// @brief 距离
  double distance;
};

typedef std::vector<LoopClosureResult> LoopClosureResultList;

/**
 * @brief The LoopClosureDetector class 激光雷达闭环检测器
 */
class LoopClosureDetector {
 public:
  typedef std::shared_ptr<LoopClosureDetector> Ptr;

  /**
   * @brief LoopClosureDetector 构造函数
   */
  LoopClosureDetector();

  /**
   * @brief LoopClosureDetector 构造函数
   * @param _loop_closure_searching_distance
   * @param _loop_closure_distance_interval
   * @param _loop_closure_score_threshold
   * @param _mono_layer_motion
   */
  LoopClosureDetector(double _loop_closure_searching_distance,
                      double _loop_closure_distance_interval,
                      double _loop_closure_score_threshold,
                      bool _mono_layer_motion);

  /**
   * @brief SetLidarMapSession 设置建图任务
   * @param session
   */
  void SetLidarMapSession(const std::shared_ptr<LidarMapSession>& _session);

  /**
   * @brief DetectLoopClosure 闭环检测 (外部调用)
   * @return
   */
  LoopClosureResult DetectLoopClosure(int detect_unit_id);

  /**
   * @brief DetectLoopClosureAll 闭环检测所有的结果
   * @param detect_unit_id
   * @return
   */
  LoopClosureResultList DetectLoopClosureAll(int detect_unit_id);

  /**
   * @brief 点云配准验证
   *
   * @param _cur_id
   * @param _history_id
   * @param _transform
   * @param _score
   * @return true
   * @return false
   */
  bool CloudRevalidation(int _cur_id, int _history_id,
                         Eigen::Matrix4d& _transform, float& _score);

  /**
   * @brief 点云配准验证
   *
   * @param _target_cloud
   * @param _source_cloud
   * @param _predict_matrix
   * @param _result_matrix
   * @param _score
   * @return true
   * @return false
   */
  bool CloudRevalidation(CloudTypePtr _target_cloud, CloudTypePtr _source_cloud,
                         Eigen::Matrix4d _predict_matrix,
                         Eigen::Matrix4d& _result_matrix, float& _score);

  /**
   * @brief 基于ICP的点云配准
   *
   * @param target_cloud
   * @param source_cloud
   * @param pre_transf
   * @param _icp_transf
   * @param _score
   * @return true
   * @return false
   */
  bool ICPMatch(CloudTypePtr target_cloud, CloudTypePtr source_cloud,
                Eigen::Matrix4d pre_transf, Eigen::Matrix4d& _icp_transf,
                float& _score);

  /**
   * @brief 基于NDT的点云配准
   *
   * @param target_cloud
   * @param source_cloud
   * @param pre_transf
   * @param _icp_transf
   * @param _score
   * @return true
   * @return false
   */
  bool NDTMatch(CloudTypePtr target_cloud, CloudTypePtr source_cloud,
                Eigen::Matrix4d pre_transf, Eigen::Matrix4d& _icp_transf,
                float& _score);

  /**
   * @brief 基于GICP的点云配准
   *
   * @param target_cloud
   * @param source_cloud
   * @param pre_transf
   * @param _icp_transf
   * @param _score
   * @return true
   * @return false
   */
  bool GICPMatch(CloudTypePtr target_cloud, CloudTypePtr source_cloud,
                 Eigen::Matrix4d pre_transf, Eigen::Matrix4d& _icp_transf,
                 float& _score);

 private:
  /**
   * @brief DetectLoopClosureCandidatesByDistance 根据距离选取闭环候选帧
   * @param cur_key_scan_id
   * @return
   */
  std::vector<LoopCandidate> DetectLoopClosureCandidatesByDistance(
      int cur_key_scan_id);

  /**
   * @brief DrawLoopClosureCloud 绘制显示的闭环点云
   * @param target_cloud
   * @param source_cloud
   * @param init_pose
   * @param aligned_pose
   */
  void DrawLoopClosureCloud(CloudTypePtr target_cloud,
                            CloudTypePtr source_cloud,
                            Eigen::Matrix4d& init_pose,
                            Eigen::Matrix4d& aligned_pose);

 private:
  /// @brief 历史帧搜索半径
  double loop_closure_searching_distance_;
  /// @brief 历史帧距离间隔
  double loop_closure_distance_interval_;
  /// @brief 闭环ICP分数阈值
  double loop_closure_score_threshold_;
  /// @brief 是否为单层运动
  bool mono_layer_motion_;

  /// @brief kd tree 用于搜索最近历史帧
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_history_pose_;

  /// @brief 当前建图任务
  std::shared_ptr<LidarMapSession> lidar_map_session_ptr_;

  /// @brief 闭环点云
  ColorPointCloudPtr loop_closure_cloud_;
};

}  // namespace multi_sensor_mapping

#endif
