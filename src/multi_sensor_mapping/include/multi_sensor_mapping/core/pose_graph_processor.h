#ifndef MSM_POSE_GRAPH_PROCESSOR_H
#define MSM_POSE_GRAPH_PROCESSOR_H

#include <functional>
#include <memory>
#include <thread>

#include "multi_sensor_mapping/backend/pose_graph_information.h"
#include "multi_sensor_mapping/utils/utils_error.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

class LidarMapSession;
class LoopClosureDetector;
class SensorParams;
class ExtrinsicParams;
class LidarImuMappingParams;
class PoseGraphManager;
class ParamSet;

/**
 * @brief  状态
 *
 */
enum PoseGraphState {
  PGS_UNINITIALIZED = 0,      //未初始化
  PGS_IDLE,                   // 空闲
  PGS_AUTO_LOOP_DETECTION,    // 自动闭环检测
  PGS_MANUAL_LOOP_DETECTION,  // 手动闭环检测
  PGS_OPTIMIZATION,           // 优化
  PGS_SAVE,                   // 保存
  PGS_POST_PROCESS,           // 后处理
  PGS_AUTO_BACKEND            // 自动后端
};

/**
 * @brief 局部匹配命令
 *
 */
enum LocalMatchCmd {
  LMC_NONE = 0,  // 不使用
  LMC_ADJUST,    // 调整
  LMC_MATCH,     // 匹配
  LMC_UPDATE     // 更新
};

/**
 * @brief 位姿图处理器
 *
 */
class PoseGraphProcessor {
 public:
  /**
   * @brief Construct a new Pose Graph Processor object 构造函数
   *
   */
  PoseGraphProcessor();

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
   * @param _session_path
   */
  bool SetBaseMapSession(std::string _session_path);

  /**
   * @brief Set the Map Session object 设置地图任务
   *
   * @param _session
   */
  bool SetBaseMapSession(const std::shared_ptr<LidarMapSession>& _session);

  /**
   * @brief 开始自动闭环检测
   *
   */
  void StartAutoLoopClosureDetection();

  /**
   * @brief 开始后端优化
   *
   */
  void StartBackendOptimization();

  /**
   * @brief 开始手动调整闭环
   *
   * @param _lc_id
   */
  void StartManualLoopClosureDetection(int _lc_id);

  /**
   * @brief 调整local match
   *
   * @param _lc_id
   * @param _x
   * @param _y
   * @param _z
   * @param _roll
   * @param _pitch
   * @param _yaw
   */
  void AdjustLocalMatch(int _lc_id, double _x, double _y, double _z,
                        double _roll, double _pitch, double _yaw);

  /**
   * @brief 匹配local match
   *
   * @param _lc_id
   * @param _x
   * @param _y
   * @param _z
   * @param _roll
   * @param _pitch
   * @param _yaw
   */
  void MatchLocalMatch(int _lc_id, double _x, double _y, double _z,
                       double _roll, double _pitch, double _yaw);

  /**
   * @brief 更新local match
   *
   * @param _lc_id
   */
  void UpdateLocalMatch(int _lc_id);

  /**
   * @brief 开始后处理
   *
   */
  void StartPostProcess();

  /**
   * @brief 调整切片参数
   *
   * @param _delta_radius
   * @param _delta_height
   * @param _delta_thickness
   */
  void AdjustSliceParam(double _delta_radius, double _delta_height,
                        double _delta_thickness);

  /**
   * @brief 调整切片参数2
   *
   * @param _slice_radius
   * @param _slice_height
   * @param _slice_thickness
   */
  void AdjustSliceParam2(double _slice_radius, double _slice_height,
                         double _slice_thickness);

  /**
   * @brief 调整地图yaw角
   *
   * @param _delta_yaw
   */
  void AdjustMapYaw(double _delta_yaw);

  /**
   * @brief 调整地图分辨率
   *
   * @param _resolution
   */
  void AdjustMapResolution(double _resolution);

  /**
   * @brief 保存地图
   *
   * @param _session_path
   */
  void SaveLidarMapSession(std::string _save_path = "");

  /**
   * @brief 更新激光建图任务
   *
   */
  void UpdateMapSession();

  /**
   * @brief 是否正在运行
   *
   * @return true
   * @return false
   */
  bool IsRunning();

  /**
   * @brief 注册异常回调函数
   *
   * @param _cb_excep
   */
  void RegExceptionCallback(
      const std::function<void(const MSMError&)>& _cb_excep);

  /**
   * @brief 注册pose graph 回调函数
   *
   * @param _cb_pose_graph
   */
  void RegPoseGraphInfoCallback(
      const std::function<void(const PoseGraph&)>& _cb_pose_graph);

  /**
   * @brief 注册全局地图回调函数
   *
   * @param _cb_global_cloud
   */
  void RegGlobalCloudCallback(
      const std::function<void(const CloudTypePtr&)>& _cb_global_cloud);

  /**
   * @brief 局部匹配信息回调函数
   *
   * @param _cb_local_match
   */
  void RegLocalMatchInfoCallback(
      const std::function<void(const LocalMatchInformation&)>& _cb_local_match);

  /**
   * @brief 异常处理
   *
   * @param _error
   */
  void RunException(const MSMError& _error);

  /**
   * @brief 处理状态回调函数
   *
   * @param _cb_process_state
   */
  void RegProcessState(
      const std::function<void(const ProcessState&)>& _cb_process_state);

 private:
  /**
   * @brief 循环处理
   *
   */
  void LoopProcess();

  /**
   * @brief 自动闭环检测
   *
   * @param _session
   */
  void AutoLoopDetection(const std::shared_ptr<LidarMapSession>& _session);

  /**
   * @brief 后端优化
   *
   */
  void BackendOptimization();

 private:
  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;

  /// @brief 地图分辨率
  double map_resolution_;

  /// @brief 状态
  PoseGraphState pose_graph_state_;

  /// @brief 处理线程
  std::thread process_thread_;

  /// @brief 基础建图任务
  std::shared_ptr<LidarMapSession> base_map_session_;
  /// @brief 任务缓存路径
  std::string session_cache_path_;
  /// @brief 任务保存路径
  std::string session_save_path_;

  /// @brief 更新地图任务标志位
  bool update_base_session_flag_;
  /// @brief 更新闭环信息
  bool update_loop_closure_info_flag_;
  /// @brief 被选中的闭环ID
  int selected_loop_closure_id_;

  /// @brief 更新切片参数标志位
  int update_slice_param_flag_;
  /// @brief 切片参数
  Eigen::Vector3d delta_slice_param_;
  /// @brief 切片参数
  Eigen::Vector3d slice_param_;

  /// @brief 更新地图yaw标志位
  bool update_map_yaw_flag_;
  /// @brief 地图调整参数
  double delta_map_yaw_;
  /// @brief 更新任务标志位
  bool update_session_flag_;

  /// @brief local match命令
  LocalMatchCmd local_match_cmd_;
  /// @brief 调整后端source点云姿态
  Eigen::Matrix4d adjusted_local_source_pose_;

  /// @brief 传感器参数
  std::shared_ptr<SensorParams> sensor_params_;
  /// @brief 外参参数
  std::shared_ptr<ExtrinsicParams> extrinsic_params_;
  /// @brief 建图参数
  std::shared_ptr<LidarImuMappingParams> mapper_params_;

  /// @brief 闭环检测器
  std::shared_ptr<LoopClosureDetector> loop_closure_detector_;
  /// @brief 因子图管理器
  std::shared_ptr<PoseGraphManager> pose_graph_manager_;

  // /// @brief 异常回调函数
  std::function<void(const MSMError&)> cb_exception_;
  /// @brief 位姿图回调
  std::function<void(const PoseGraph&)> cb_pose_graph_;
  /// @brief 全局地图回调
  std::function<void(const CloudTypePtr&)> cb_global_cloud_;
  /// @brief 局部匹配回调
  std::function<void(const LocalMatchInformation&)> cb_local_match_info_;
  /// @brief 处理状态回调
  std::function<void(const ProcessState&)> cb_process_state_;
};

}  // namespace multi_sensor_mapping

#endif