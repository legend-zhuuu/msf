#ifndef MSM_TINY_POSE_GRAPH_PROCESSOR_H
#define MSM_TINY_POSE_GRAPH_PROCESSOR_H

#include <functional>
#include <memory>
#include <thread>

#include "multi_sensor_mapping/backend/pose_graph_information.h"

namespace multi_sensor_mapping {

class LidarMapSession;
class ParamSet;
class PoseGraphParams;
class LoopClosureDetector;
class PoseGraphManager;

/// @brief 状态
enum TinyPoseGraphState {
  TPGS_IDLE = 0,      // 空闲
  TPGS_LOOP_CLOSURE,  // 闭环
  TPGS_ADJUST,        // 调整
  TPGS_SAVE           // 保存
};

/**
 * @brief 轻量级位姿图处理器(适用202项目)
 *
 */
class TinyPoseGraphProcessor {
 public:
  /**
   * @brief Construct a new Tiny Pose Graph Processor object
   *
   */
  TinyPoseGraphProcessor();

  /**
   * @brief Set the Params object 设置参数
   *
   * @param _param_set
   */
  void SetParams(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief 初始化
   *
   */
  void Init();

  /**
   * @brief 开始
   *
   */
  void Start();

  /**
   * @brief 停止
   *
   */
  void Stop();

  /**
   * @brief Set the Map Session object 设置地图任务
   *
   * @param _session
   */
  void SetBaseMapSession(const std::shared_ptr<LidarMapSession>& _session);

  /**
   * @brief Set the Cache Path object
   *
   * @param _cache_path
   */
  void SetCachePath(std::string _cache_path);

  /**
   * @brief Set the Map Rotation object 设置地图旋转
   *
   * @param _rotation
   */
  void SetMapRotation(const Eigen::Quaterniond& _rotation);

  /**
   * @brief Save map 保存地图
   *
   */
  void SaveMap();

  /**
   * @brief 注册进度回调函数
   *
   * @param _cb_progress
   */
  void RegProcessCallback(
      const std::function<void(const double&)>& _cb_progress);

  /**
   * @brief 注册pose graph 回调函数
   *
   * @param _cb_pose_graph
   */
  void RegPoseGraphInfoCallback(
      const std::function<void(const PoseGraph&)>& _cb_pose_graph);

  /**
   * @brief 局部匹配信息回调函数
   *
   * @param _cb_local_match
   */
  void RegLocalMatchInfoCallback(
      const std::function<void(const LocalMatchInformation&)>& _cb_local_match);

  /**
   * @brief 全局点云回调函数
   *
   * @param _cb_global_cloud
   */
  void RegGlobalCloudCallback(
      const std::function<void(const CloudTypePtr&)>& _cb_global_cloud);

  /**
   * @brief 注册关键帧回调函数
   *
   * @param _cb_keyframe_pose
   */
  void RegKeyFramePosesCallback(
      const std::function<void(const CloudTypePtr&)>& _cb_keyframe_cloud);

  /**
   * @brief 是否在运行
   *
   * @return true
   * @return false
   */
  bool IsRunning();

 private:
  /**
   * @brief 后端优化处理函数
   *
   */
  void BackendProcess();

  /**
   * @brief 处理进度
   *
   * @param _progress
   */
  void RunPutProgress(const double& _progress);

  /**
   * @brief 重置
   *
   */
  void Reset();

 private:
  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief 工作状态
  TinyPoseGraphState state_;

  /// @brief 闭环检测跳帧数
  int loop_closure_skip_frame_;
  /// @brief 缓存路径
  std::string cache_path_;

  /// @brief 位姿图参数
  std::shared_ptr<PoseGraphParams> pose_graph_params_;

  /// @brief 激光建图任务
  std::shared_ptr<LidarMapSession> lidar_session_;
  /// @brief 闭环检测器
  std::shared_ptr<LoopClosureDetector> loop_closure_detector_;
  /// @brief 因子图管理器
  std::shared_ptr<PoseGraphManager> pose_graph_manager_;
  /// @brief 地图全局旋转
  Eigen::Quaterniond map_global_rotation_;

  /// @brief 处理线程
  std::thread process_thread_;

  /// @brief 建图进度回调函数
  std::function<void(const double&)> cb_put_progress_;
  /// @brief 位姿图回调
  std::function<void(const PoseGraph&)> cb_pose_graph_;
  /// @brief 局部匹配回调
  std::function<void(const LocalMatchInformation&)> cb_local_match_info_;
  /// @brief 全局点云回调函数
  std::function<void(const CloudTypePtr&)> cb_global_cloud_;
  /// @brief 关键帧姿态回调函数
  std::function<void(const CloudTypePtr&)> cb_keyframe_cloud_;
};

}  // namespace multi_sensor_mapping

#endif