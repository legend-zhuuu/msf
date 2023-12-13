#ifndef MSM_LIDAR_MAPPER_BASE_H
#define MSM_LIDAR_MAPPER_BASE_H

#include <functional>
#include <memory>
#include <thread>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_error.h"

namespace multi_sensor_mapping {

class SensorParams;
class ExtrinsicParams;
class DataConverter;
class ParamSet;
class LidarMapSession;

/**
 * @brief 激光建图基类
 *
 */
class LidarMapperBase {
 public:
  /**
   * @brief Construct a new Lidar Mapper Base object 构造函数
   *
   */
  LidarMapperBase();

  /**
   * @brief 设置数据包路径
   *
   * @param _bag_path
   */
  void SetBagPath(std::string _bag_path);

  /**
   * @brief 设置建图片段
   *
   * @param _start_time
   * @param _map_durr
   */
  void SetMappingPeriod(double _start_time = 0, double _map_durr = -1);

  /**
   * @brief Set the Params object 设置参数集合
   *
   * @param _param_set
   */
  virtual void SetParams(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief 初始化
   *
   */
  virtual void Init();

  /**
   * @brief 开始建图
   *
   */
  virtual void Start();

  /**
   * @brief 结束建图
   *
   */
  virtual void Stop();

  /**
   * @brief 注册异常回调函数
   *
   * @param cb_excep
   */
  void RegExceptionCallback(
      const std::function<void(const MSMError&)>& cb_excep);

  /**
   * @brief 注册激光位姿回调函数
   *
   * @param _cb_lidar_pose
   */
  void RegLidarPoseCallback(
      const std::function<void(const PoseData&)>& _cb_lidar_pose);

  /**
   * @brief 注册关键帧回调函数
   *
   * @param _cb_key_frame
   */
  void RegKeyFrameCallback(
      const std::function<void(const CloudTypePtr&, const PoseData&)>&
          _cb_key_frame);

  /**
   * @brief 注册普通帧回调函数
   *
   * @param _cb_frame
   */
  void RegFrameCallback(const std::function<void(const CloudTypePtr&,
                                                 const PoseData&)>& _cb_frame);

  /**
   * @brief 建图进度回调函数
   *
   * @param _cb_progress
   */
  void RegPutMappingProgress(
      const std::function<void(const double&)>& _cb_progress);

  /**
   * @brief 地图任务回调函数
   *
   * @param _cb_session
   */
  void RegPutMapSession(
      const std::function<void(const std::shared_ptr<LidarMapSession>&)>&
          _cb_session);

  /**
   * @brief 异常处理
   *
   * @param _error
   */
  void RunException(const MSMError& _error);

  /**
   * @brief 处理激光姿态
   *
   * @param _lidar_pose
   */
  void RunPutLidarPose(const PoseData& _lidar_pose);

  /**
   * @brief 处理关键帧
   *
   * @param _cloud
   * @param _pose
   */
  void RunPutKeyFrame(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 处理普通帧
   *
   * @param _cloud
   * @param _pose
   */
  void RunPutFrame(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 处理建图进度
   *
   * @param _progress
   */
  void RunPutMappingProgress(const double& _progress);

  /**
   * @brief 是否正在运行
   *
   * @return true
   * @return false
   */
  bool IsRunning();

 protected:
  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;

  /// @brief 数据包路径
  std::string bag_path_;
  /// @brief 数据包起始时间
  double bag_start_time_;
  /// @brief 数据包长度
  double bag_durr_;

  /// @brief 传感器参数
  std::shared_ptr<SensorParams> sensor_params_;
  /// @brief 外参参数
  std::shared_ptr<ExtrinsicParams> extrinsic_params_;

  /// @brief 数据转换器
  std::shared_ptr<DataConverter> data_converter_;

  /// @brief 处理线程
  std::thread process_thread_;

  /// @brief 异常回调函数
  std::function<void(const MSMError&)> cb_exception_;
  /// @brief 姿态回调函数
  std::function<void(const PoseData&)> cb_put_lidar_pose_;
  /// @brief 关键帧回调函数
  std::function<void(const CloudTypePtr&, const PoseData&)> cb_put_key_frame_;
  /// @brief 普通帧的回调函数
  std::function<void(const CloudTypePtr&, const PoseData&)> cb_put_frame_;
  /// @brief 建图进度回调函数
  std::function<void(const double&)> cb_put_mapping_progress_;
  /// @brief 激光建图任务回调函数
  std::function<void(const std::shared_ptr<LidarMapSession>&)> cb_put_session_;
};

}  // namespace multi_sensor_mapping

#endif