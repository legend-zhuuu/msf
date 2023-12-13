#ifndef MSM_SDK_BACKEND_PROCESS_H
#define MSM_SDK_BACKEND_PROCESS_H

#include <functional>
#include <memory>
#include <thread>

#include "msm_types.h"

namespace multi_sensor_mapping {
class PoseGraphProcessor;
class LidarMapSession;
}  // namespace multi_sensor_mapping

namespace msm_sdk {

/**
 * @brief 后端处理API
 *
 */
class APIBackendProcess {
 public:
  /**
   * @brief Construct a new APIBackendProcess object 构造函数
   *
   */
  APIBackendProcess();

  /**
   * @brief Set the Mapping Param object 设置建图参数
   *
   * @param _scene
   * @param use_loop_closure
   */
  void SetMappingParam(MappingScene _scene, bool use_loop_closure);

  /**
   * @brief 初始化
   *
   * @param _param_path
   * @return true
   * @return false
   */
  bool Init(std::string _param_path);

  /**
   * @brief 开始
   *
   * @param _session_path
   */
  void Start(std::string _session_path);

  /**
   * @brief 开始
   *
   * @param _session
   */
  void Start(
      const std::shared_ptr<multi_sensor_mapping::LidarMapSession>& _session);

  /**
   * @brief 结束
   *
   */
  void Stop();

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
   * @brief 调整local match (单位:米, 角度)
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
   * @brief 匹配local match (单位:米, 角度)
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
   * @brief 开始地图后处理
   *
   */
  void StartPostProcess();

  /**
   * @brief 更新地图
   *
   */
  void UpdateMapSession();

  /**
   * @brief 调整切片参数
   *
   * @param _delta_radius
   * @param _delta_height
   * @param _delta_thickness
   */
  void AdjustDeltaSliceParam(double _delta_radius, double _delta_height,
                             double _delta_thickness);

  /**
   * @brief 调整切片参数
   *
   * @param _radius
   * @param _height
   * @param _thickness
   */
  void AdjustSliceParam(double _radius, double _height, double _thickness);

  /**
   * @brief 调整地图yaw角(单位: 角度)
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
  void SaveLidarMapSession(std::string _save_path);

  /**
   * @brief 注册位姿图回调函数
   *
   * @param _cb_graph
   */
  void RegGraphCallback(const std::function<void(const Graph&)>& _cb_graph);

  /**
   * @brief 注册闭环回调函数
   *
   * @param _cb_loop_closure
   */
  void RegLoopClosureCallback(
      const std::function<void(const LoopClosureInfo&)>& _cb_loop_closure);

  /**
   * @brief 注册全局地图回调函数
   *
   * @param _cb_global_map
   */
  void RegGlobalMapCallback(
      const std::function<void(const StampedCloud&)>& _cb_global_map);

  /**
   * @brief 注册处理状态回调函数
   *
   * @param _cb_process_state
   */
  void RegProcessStateCallback(
      const std::function<void(const bool&)>& _cb_process_state);

 private:
  /**
   * @brief 核心处理函数
   *
   */
  void Process();

 private:
  /// @brief 激光建图
  std::shared_ptr<multi_sensor_mapping::PoseGraphProcessor> backend_;

  /// @brief 退出线程标志位
  bool exit_process_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;

  /// @brief 处理线程
  std::thread process_thead_;

  /// @brief 位姿图回调函数
  std::function<void(const Graph&)> cb_graph_;
  /// @brief 闭环信息回调函数
  std::function<void(const LoopClosureInfo&)> cb_loop_closure_;
  /// @brief 全局地图回调函数
  std::function<void(const StampedCloud&)> cb_global_cloud_;
  /// @brief 处理状态回调函数
  std::function<void(const bool&)> cb_process_state_;
};
}  // namespace msm_sdk

#endif