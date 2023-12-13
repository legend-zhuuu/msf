#ifndef MSM_SDK_LOW_COST_MAPPING_H
#define MSM_SDK_LOW_COST_MAPPING_H

#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "msm_types.h"

namespace multi_sensor_mapping {
class FastLIOMapper;
class LidarMapper2DVisualizer;
class LidarMapSession;
class TinyPoseGraphProcessor;
class LidarDetector;
}  // namespace multi_sensor_mapping

namespace msm_sdk {

/**
 * @brief 低功耗建图API
 *
 */
class APILowCostMapping {
 public:
  /**
   * @brief Construct a new APILowCostMapping object
   *
   */
  APILowCostMapping();

  /**
   * @brief Set the Map Cache Path object 设置地图缓存路径
   *
   * @param _path
   */
  void SetMapCachePath(std::string _path);

  /**
   * @brief Set the Visualization Resolution object 设置可视化分辨率
   *
   * @param _resolution
   */
  void SetVisualizationResolution(float _resolution);

  /**
   * @brief Set the Direction Offset object 设置方向偏置角
   *
   * @param _direction_offset
   */
  void SetDirectionOffset(float _direction_offset);

  /**
   * @brief 初始化
   *
   */
  bool Init(std::string _param_path);

  /**
   * @brief 开始
   *
   */
  void Start();

  /**
   * @brief 开始后端
   *
   */
  void StartBackend();

  /**
   * @brief 结束
   *
   */
  void Stop();

  /**
   * @brief 可视化图像回调函数
   *
   * @param _cb_img
   */
  void RegVisualizationImgCallback(
      const std::function<void(const cv::Mat&)>& _cb_img);

  /**
   * @brief 进度回调函数
   *
   * @param _cb_progress
   */
  void RegProgressCallback(
      const std::function<void(const double&)>& _cb_progress);

  /**
   * @brief 注册定位姿态回调函数
   *
   * @param _cb_location_pose
   */
  void RegLocationPoseCallback(
      const std::function<void(const StampedPose&)>& _cb_location_pose);

  /**
   * @brief 注册2D定位回调函数
   *
   * @param _cb_location_2d_pose
   */
  void RegLocation2dPoseCallback(
      const std::function<void(const Eigen::Vector3f&)>& _cb_location_2d_pose);

  /**
   * @brief 注册像素定位回调函数
   *
   * @param _cb_location_pixel_pose
   */
  void RegLocationPixPoseCallback(
      const std::function<void(const Eigen::Vector3f&)>&
          _cb_location_pixel_pose);

  /**
   * @brief 注册关键帧位置回调函数
   *
   * @param _cb_key_frame_pixel_pose
   */
  void RegKeyFramePixelPoseCallback(
      const std::function<void(const std::vector<Eigen::Vector2f>&)>&
          _cb_key_frame_pixel_pose);

  /**
   * @brief 注册标签像素位置回调函数
   *
   * @param _cb_tag_pixel_pose
   */
  void RegTagPixelPoseCallback(
      const std::function<void(const Eigen::Vector3f&)>& _cb_tag_pixel_pose);

  /**
   * @brief 注册传感器异常回调函数
   *
   * @param _cb_sensor_abnormal
   */
  void RegSensorAbnormalCallback(
      const std::function<void(const bool&)>& _cb_sensor_abnormal);

 private:
  /**
   * @brief 建图处理函数
   *
   */
  void MappingProcess();

  /**
   * @brief 可视化处理函数
   *
   */
  void VisualizationProcess();

  /**
   * @brief 关键帧选择
   *
   * @param _cloud
   * @param _position
   * @param _rotation
   */
  void KeyframeSelection(double _timestamp, const CloudType& _cloud,
                         const Eigen::Vector3d& _position,
                         const Eigen::Quaterniond& _rotation);

  /**
   * @brief 重置
   *
   */
  void Reset();

 private:
  /// @brief 建图器
  std::shared_ptr<multi_sensor_mapping::FastLIOMapper> mapper_;
  /// @brief 后端处理器
  std::shared_ptr<multi_sensor_mapping::TinyPoseGraphProcessor>
      pose_graph_processor_;
  /// @brief 激光检测器
  std::shared_ptr<multi_sensor_mapping::LidarDetector> lidar_detector_;

  /// @brief 激光建图任务
  std::shared_ptr<multi_sensor_mapping::LidarMapSession> lidar_session_;
  /// @brief 激光建图可视化
  std::shared_ptr<multi_sensor_mapping::LidarMapper2DVisualizer> visualizer_;

  /// @brief 退出线程标志位
  bool exit_process_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 地图缓存路径
  std::string map_cache_path_;

  /// @brief 建图线程
  std::thread lidar_mapping_thread_;
  /// @brief 可视化线程
  std::thread visualization_thread_;

  /// @brief 可视化图像回调函数
  std::function<void(const cv::Mat&)> cb_visualization_img_;
  /// @brief 进度回调函数
  std::function<void(const double)> cb_progress_;
  /// @brief  定位姿态回调函数
  std::function<void(const StampedPose&)> cb_location_pose_;
  /// @brief 2D定位姿态回调函数(x,y,yaw)
  std::function<void(const Eigen::Vector3f&)> cb_location_2d_pose_;
  /// @brief 定位像素姿态回调函数(x,y,yaw)
  std::function<void(const Eigen::Vector3f&)> cb_location_pixel_pose_;
  /// @brief 关键帧位姿回调函数 (x,y)
  std::function<void(const std::vector<Eigen::Vector2f>&)>
      cb_key_frame_pixel_pose_;
  /// @brief tag位姿回调函数 (x,y,yaw)
  std::function<void(const Eigen::Vector3f&)> cb_tag_pixel_pose_;
  /// @brief 传感器异常回调函数
  std::function<void(const bool&)> cb_sensor_abnormal_;
};
}  // namespace msm_sdk

#endif