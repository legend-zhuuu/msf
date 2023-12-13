#ifndef MSM_SDK_LOW_COST_LOCATION_H
#define MSM_SDK_LOW_COST_LOCATION_H

#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "msm_types.h"

namespace multi_sensor_mapping {
class FastLIOMapper;
class LidarMapper2DVisualizer;
class TinyLidarLocator;
class LidarDetector;
}  // namespace multi_sensor_mapping

namespace msm_sdk {

/**
 * @brief 低功耗定位API
 *
 */
class APILowCostLocation {
 public:
  /**
   * @brief Construct a new APILowCostLocation object
   *
   */
  APILowCostLocation();

  /**
   * @brief Set the Map Path object 设置地图路径
   *
   * @param _map_path
   */
  void SetMapPath(std::string _map_path);

  /**
   * @brief Set the Visualization Resolution object 设置可视化分辨率
   *
   * @param _resolution
   */
  void SetVisualizationResolution(float _resolution);

  /**
   * @brief Set the Initial Pose object 设置初始位姿
   *
   * @param _position
   * @param _rotation
   */
  void SetInitialPose(const Eigen::Vector3d& _position,
                      const Eigen::Quaterniond& _rotation);

  /**
   * @brief Set the Direction Offset object 设置方向偏置
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
   * @brief 结束
   *
   */
  void Stop();

  /**
   * @brief 注册可视化图像回调函数
   *
   * @param _cb_img
   */
  void RegVisualizationImgCallback(
      const std::function<void(const cv::Mat&)>& _cb_img);

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
   * @brief 注册标记点像素坐标回调函数
   *
   * @param _cb_mark_points_pixel_pose
   */
  void RegMarkPointsPixPoseCallback(
      const std::function<void(const std::vector<Eigen::Vector2f>&)>&
          _cb_mark_points_pixel_pose);

  /**
   * @brief 注册高亮标记点像素回调函数
   *
   * @param _cb_highlight_mark_point_pixel_pose
   */
  void RegHighlightMarkPointPixPoseCallback(
      const std::function<void(const Eigen::Vector2f&)>&
          _cb_highlight_mark_point_pixel_pose);

  /**
   * @brief Get the Map Resolution object 获取地图分辨率
   *
   * @return float
   */
  float GetMapResolution();

  /**
   * @brief 输入标记位置
   *
   * @param _pos_x
   * @param _pos_y
   */
  void InputMarkingPose(double _pos_x, double _pos_y);

  /**
   * @brief 输入高亮的标记位置
   *
   * @param _pos_x
   * @param _pos_y
   */
  void InputHighLightMarkingPose(double _pos_x, double _pos_y);

 private:
  /**
   * @brief 定位处理函数
   *
   */
  void LocationProcess();

  /**
   * @brief 可视化处理函数
   *
   */
  void VisualizationProcess();

  /**
   * @brief 重置
   *
   */
  void Reset();

 private:
  /// @brief 建图器
  std::shared_ptr<multi_sensor_mapping::FastLIOMapper> mapper_;
  /// @brief 定位器
  std::shared_ptr<multi_sensor_mapping::TinyLidarLocator> locator_;
  /// @brief 激光检测器
  std::shared_ptr<multi_sensor_mapping::LidarDetector> detector_;
  /// @brief 激光建图可视化
  std::shared_ptr<multi_sensor_mapping::LidarMapper2DVisualizer> visualizer_;

  /// @brief 退出线程标志位
  bool exit_process_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief tag初始化标志位
  bool tag_init_done_flag_;

  /// @brief 参考tag姿态
  std::vector<Eigen::Matrix4d> ref_lidar_tag_pose_;

  /// @brief 缓存路径
  std::string cache_path_;

  /// @brief 定位线程
  std::thread location_thread_;
  /// @brief 可视化线程
  std::thread visualization_thread_;

  /// @brief 标记位置集合
  std::vector<Eigen::Vector3d> mark_pose_container_;

  /// @brief 可视化图像回调函数
  std::function<void(const cv::Mat&)> cb_visualization_img_;
  /// @brief  定位姿态回调函数
  std::function<void(const StampedPose&)> cb_location_pose_;
  /// @brief 2D定位姿态回调函数(x,y,yaw)
  std::function<void(const Eigen::Vector3f&)> cb_location_2d_pose_;
  /// @brief 定位像素姿态回调函数(x,y,yaw)
  std::function<void(const Eigen::Vector3f&)> cb_location_pixel_pose_;
  /// @brief 标记点位像素位置回调函数(x,y)
  std::function<void(const std::vector<Eigen::Vector2f>&)>
      cb_mark_points_pixel_pose_;
  /// @brief 标记点像素位置回调函数(x,y)
  std::function<void(const Eigen::Vector2f&)>
      cb_highlight_mark_point_pixel_pose_;
};
}  // namespace msm_sdk

#endif