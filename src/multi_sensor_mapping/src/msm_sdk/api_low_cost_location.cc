#include "msm_sdk/api_low_cost_location.h"

#include <atomic>
#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <thread>
#include <vector>

#include "multi_sensor_mapping/core/fast_lio_mapper.h"
#include "multi_sensor_mapping/core/lidar_detector.h"
#include "multi_sensor_mapping/core/tiny_lidar_locator.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/tiny_location_params.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_2d_visualizer.h"

using msm_sdk::APILowCostLocation;

namespace low_cost_location {

/// @brief 锁
std::mutex lio_pose_mtx;
std::mutex lio_frame_mtx;
std::mutex location_pose_mtx;
std::mutex location_cloud_mtx;
std::mutex lidar_tag_mutex;

/// @brief  LIO姿态缓存
std::vector<multi_sensor_mapping::PoseData> lio_pose_queue;
/// @brief  LIO关键帧缓存
std::vector<std::pair<CloudTypePtr, multi_sensor_mapping::PoseData> >
    lio_frame_queue;

/// @brief  定位姿态更新标志位
bool update_location_pose_flag = false;
/// @brief 关键帧定位点云标志位
bool update_location_cloud_flag = false;
/// @brief 激光tag更新标志位
bool update_lidar_tag_flag = false;
/// @brief 更新标记点姿态标志位
bool refresh_mark_points_pose_flag = false;
/// @brief  更新重点标记点标志位
bool refresh_highlight_mark_pose_flag = false;
/// @brief 上一帧关键帧时间戳
double last_key_frame_timestamp = 0;

Eigen::Vector3d highlight_mark_pose = Eigen::Vector3d(0, 0, 0);

/// @brief 定位姿态
multi_sensor_mapping::PoseData location_pose;
/// @brief 关键帧定位点云
CloudTypePtr key_frame_location_cloud(new CloudType);

/// @brief 全局匹配时间间隔
double global_registration_interval;

/// @brief 全局点云地图
CloudTypePtr global_cloud(new CloudType);
/// @brief 激光tag位置
std::vector<multi_sensor_mapping::PoseData> lidar_tag_poses;
// 可视化图像
cv::Mat visualization_img;

/// @brief 方向角偏置
float direction_offset = 180;

void PutLioPose(const multi_sensor_mapping::PoseData& _pose) {
  std::lock_guard<std::mutex> lg(lio_pose_mtx);
  lio_pose_queue.push_back(_pose);
}

void PutLioFrame(const CloudTypePtr& _cloud,
                 const multi_sensor_mapping::PoseData& _pose) {
  // static double last_key_frame_timestamp = 0;

  if (_cloud->size() < 100) {
    return;
  }

  if (_pose.timestamp - last_key_frame_timestamp >
      global_registration_interval) {
    std::lock_guard<std::mutex> lg(lio_frame_mtx);
    lio_frame_queue.push_back(std::make_pair(_cloud, _pose));
    last_key_frame_timestamp = _pose.timestamp;
  }
}

void PutLocationPose(const multi_sensor_mapping::PoseData& _pose) {
  std::lock_guard<std::mutex> lg(location_pose_mtx);
  location_pose = _pose;
  update_location_pose_flag = true;
}

void PutLocationCloud(const CloudTypePtr& _cloud) {
  std::lock_guard<std::mutex> lg(location_cloud_mtx);
  *key_frame_location_cloud = *_cloud;
  update_location_cloud_flag = true;
}

void PutLidarTagPoses(
    const std::vector<multi_sensor_mapping::PoseData>& _tag_poses) {
  std::lock_guard<std::mutex> lg(lidar_tag_mutex);
  if (!_tag_poses.empty()) {
    lidar_tag_poses = _tag_poses;
    update_lidar_tag_flag = true;
  }
}

void ResetGloablParam() {
  lio_pose_queue.clear();
  lio_frame_queue.clear();

  update_location_pose_flag = false;
  update_location_cloud_flag = false;
  update_lidar_tag_flag = false;
  refresh_mark_points_pose_flag = false;
  refresh_highlight_mark_pose_flag = false;
  last_key_frame_timestamp = 0;

  highlight_mark_pose = Eigen::Vector3d(0, 0, 0);

  lidar_tag_poses.clear();
}

}  // namespace low_cost_location

APILowCostLocation::APILowCostLocation()
    : exit_process_flag_(false),
      start_flag_(false),
      init_done_flag_(false),
      tag_init_done_flag_(false) {}

void APILowCostLocation::SetMapPath(std::string _map_path) {
  std::string map_cloud_path = _map_path;
  boost::filesystem::path map_path(_map_path);
  if (map_path.extension() != ".pcd") {
    map_cloud_path = _map_path + "/global_cloud.pcd";
    cache_path_ = _map_path;
  } else {
    cache_path_ = map_path.parent_path().string();
  }

  // 加载点云
  if (pcl::io::loadPCDFile(map_cloud_path, *low_cost_location::global_cloud) ==
      -1) {
    AWARN_F("[LowCostLocatorManager] Map Cloud Path is wrong : %s",
            map_cloud_path.c_str());
  }

  // 加载tag姿态
  std::string tag_info_path = cache_path_ + "/tag_info.yaml";
  YAML::Node tag_info_node;
  try {
    tag_info_node = YAML::LoadFile(tag_info_path);
  } catch (...) {
    AERROR_F(
        "The format of config file [ %s ] is wrong, Please check (e.g. "
        "indentation).",
        tag_info_path.c_str());
    return;
  }

  for (size_t i = 0; i < tag_info_node.size(); i++) {
    Eigen::Vector3d tag_position(tag_info_node[i]["x"].as<double>(),
                                 tag_info_node[i]["y"].as<double>(),
                                 tag_info_node[i]["z"].as<double>());

    Eigen::Quaterniond tag_rotation(tag_info_node[i]["qw"].as<double>(),
                                    tag_info_node[i]["qx"].as<double>(),
                                    tag_info_node[i]["qy"].as<double>(),
                                    tag_info_node[i]["qz"].as<double>());
    Eigen::Matrix4d tag_pose_mat = Eigen::Matrix4d::Identity();
    tag_pose_mat.block<3, 1>(0, 3) = tag_position;
    tag_pose_mat.block<3, 3>(0, 0) = tag_rotation.toRotationMatrix();
    ref_lidar_tag_pose_.push_back(tag_pose_mat);
  }

  if (!ref_lidar_tag_pose_.empty()) {
    tag_init_done_flag_ = true;
    AINFO_F("[LowCostLocatorManager] Load Tag num < %i >",
            (int)ref_lidar_tag_pose_.size());
  }
}

void APILowCostLocation::SetVisualizationResolution(float _resolution) {
  visualizer_->SetMapResolution(_resolution);
}

void APILowCostLocation::SetInitialPose(const Eigen::Vector3d& _position,
                                        const Eigen::Quaterniond& _rotation) {
  if (!init_done_flag_) {
    AWARN_F("[APILowCostLocation] Please init system before set initial pose");
    return;
  }

  multi_sensor_mapping::PoseData init_pose;
  init_pose.position = _position;
  init_pose.orientation = _rotation;

  locator_->InputInitialPose(init_pose);
}

void APILowCostLocation::SetDirectionOffset(float _direction_offset) {
  low_cost_location::direction_offset = _direction_offset;
}

bool APILowCostLocation::Init(std::string _param_path) {
  // Step1 加载参数
  std::shared_ptr<multi_sensor_mapping::ParamSet> param_set(
      new multi_sensor_mapping::ParamSet);
  if (!param_set->Load(_param_path)) {
    return false;
  }

  low_cost_location::global_registration_interval =
      param_set->GetTinyLocationParams()->registration_interval;

  // Step2 初始化里程计
  mapper_ = std::make_shared<multi_sensor_mapping::FastLIOMapper>();
  mapper_->SetParams(param_set);
  mapper_->Init();

  mapper_->RegLidarPoseCallback(
      std::bind(&low_cost_location::PutLioPose, std::placeholders::_1));
  mapper_->RegFrameCallback(std::bind(&low_cost_location::PutLioFrame,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

  // Step3 初始化定位器
  locator_ = std::make_shared<multi_sensor_mapping::TinyLidarLocator>();
  locator_->SetParams(param_set);
  locator_->Init();

  locator_->RegLocationPose(
      std::bind(&low_cost_location::PutLocationPose, std::placeholders::_1));
  // locator_->RegLocationCloud(
  //     std::bind(&low_cost_location::PutLocationCloud,
  //     std::placeholders::_1));

  // Step4 初始化tag检测器
  detector_ = std::make_shared<multi_sensor_mapping::LidarDetector>();
  detector_->SetParams(param_set);
  detector_->Init();

  detector_->RegTagPoses(
      std::bind(&low_cost_location::PutLidarTagPoses, std::placeholders::_1));

  // 设置地图
  if (!low_cost_location::global_cloud->empty()) {
    locator_->SetMapCloud(low_cost_location::global_cloud);
  } else {
    AWARN_F("[APILowCostLocation] global map is empty ");
  }

  // Step4 初始化可视化器
  visualizer_ =
      std::make_shared<multi_sensor_mapping::LidarMapper2DVisualizer>();

  init_done_flag_ = true;

  return true;
}

void APILowCostLocation::Start() {
  if (mapper_->IsRunning()) {
    AINFO_F(
        "[APILowCostLocation] LidarMapper process thread have already "
        "running");
    return;
  }

  if (locator_->IsRunning()) {
    AINFO_F(
        "[APILowCostLocation] LidarLocator process thread have already "
        "running");
    return;
  }

  exit_process_flag_ = false;

  // 开启建图线程
  mapper_->Start();
  // 开启定位线程
  locator_->Start();
  // 开启检测线程
  detector_->Start();

  location_thread_ =
      std::thread(std::bind(&APILowCostLocation::LocationProcess, this));
  visualization_thread_ =
      std ::thread(std::bind(&APILowCostLocation::VisualizationProcess, this));

  start_flag_ = true;
}

void APILowCostLocation::Stop() {
  if (!start_flag_) {
    return;
  }

  if (mapper_->IsRunning()) {
    mapper_->Stop();
  }

  if (locator_->IsRunning()) {
    locator_->Stop();
  }

  if (detector_->IsRunning()) {
    detector_->Stop();
  }

  exit_process_flag_ = true;

  // 保存路标点 (需求变化)
  // if (!mark_pose_container_.empty()) {
  //   // 使用当前时间戳命名文件夹
  //   auto now = std::time(nullptr);
  //   auto now_time = std::localtime(&now);
  //   int cur_mon = now_time->tm_mon + 1;
  //   int cur_day = now_time->tm_mday;
  //   char today_buffer[24];
  //   std::strftime(today_buffer, sizeof(today_buffer), "%Y-%m-%d-%H-%M-%S",
  //                 now_time);
  //   std::string time_str(today_buffer);
  //   std::string mark_pose_file_name =
  //       cache_path_ + "/mark_points(" + time_str + ").txt";
  //   std::ofstream pose_file;
  //   pose_file.open(mark_pose_file_name);

  //   for (size_t i = 0; i < mark_pose_container_.size(); i++) {
  //     pose_file << (int)i << " " << mark_pose_container_[i](0) << " "
  //               << mark_pose_container_[i](1) << "\n";
  //   }
  //   pose_file.close();
  //   AINFO_F("[APILowCostLocation] Save mark points at < %s >",
  //           mark_pose_file_name.c_str());
  // }

  location_thread_.join();
  visualization_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;

  Reset();

  AINFO_F("[APILowCostLocation] Low cost Location process thread stopped");
}

void APILowCostLocation::RegVisualizationImgCallback(
    const std::function<void(const cv::Mat&)>& _cb_img) {
  cb_visualization_img_ = _cb_img;
}

void APILowCostLocation::RegLocationPoseCallback(
    const std::function<void(const StampedPose&)>& _cb_location_pose) {
  cb_location_pose_ = _cb_location_pose;
}

void APILowCostLocation::RegLocation2dPoseCallback(
    const std::function<void(const Eigen::Vector3f&)>& _cb_location_2d_pose) {
  cb_location_2d_pose_ = _cb_location_2d_pose;
}

void APILowCostLocation::RegLocationPixPoseCallback(
    const std::function<void(const Eigen::Vector3f&)>&
        _cb_location_pixel_pose) {
  cb_location_pixel_pose_ = _cb_location_pixel_pose;
}

void APILowCostLocation::RegMarkPointsPixPoseCallback(
    const std::function<void(const std::vector<Eigen::Vector2f>&)>&
        _cb_mark_points_pixel_pose) {
  cb_mark_points_pixel_pose_ = _cb_mark_points_pixel_pose;
}

void APILowCostLocation::RegHighlightMarkPointPixPoseCallback(
    const std::function<void(const Eigen::Vector2f&)>&
        _cb_highlight_mark_point_pixel_pose) {
  cb_highlight_mark_point_pixel_pose_ = _cb_highlight_mark_point_pixel_pose;
}

float APILowCostLocation::GetMapResolution() {
  return visualizer_->GetResolution();
}

void APILowCostLocation::InputMarkingPose(double _pos_x, double _pos_y) {
  mark_pose_container_.push_back(Eigen::Vector3d(_pos_x, _pos_y, 0));
  low_cost_location::refresh_mark_points_pose_flag = true;
}

void APILowCostLocation::InputHighLightMarkingPose(double _pos_x,
                                                   double _pos_y) {
  low_cost_location::highlight_mark_pose = Eigen::Vector3d(_pos_x, _pos_y, 1.0);
  low_cost_location::refresh_highlight_mark_pose_flag = true;
}

void APILowCostLocation::LocationProcess() {
  while (!exit_process_flag_) {
    // 处理激光帧
    {
      std::lock_guard<std::mutex> lg(low_cost_location::lio_frame_mtx);
      if (!low_cost_location::lio_frame_queue.empty()) {
        auto frame_info = low_cost_location::lio_frame_queue.back();
        locator_->InputKeyFrame(frame_info.first, frame_info.second);
        low_cost_location::lio_frame_queue.clear();
      }
    }

    // 处理激光里程计数据
    {
      std::lock_guard<std::mutex> lg(low_cost_location::lio_pose_mtx);
      if (!low_cost_location::lio_pose_queue.empty()) {
        auto lio_pose = low_cost_location::lio_pose_queue.back();
        locator_->InputLIOPose(lio_pose);
        low_cost_location::lio_pose_queue.clear();
      }
    }

    // 处理激光tag
    {
      std::lock_guard<std::mutex> lg(low_cost_location::lidar_tag_mutex);
      if (low_cost_location::update_lidar_tag_flag) {
        if (tag_init_done_flag_ &&
            !low_cost_location::lidar_tag_poses.empty()) {
          // 检测到的tag姿态
          Eigen::Matrix4d detected_tag_pose = Eigen::Matrix4d::Identity();
          detected_tag_pose.block<3, 1>(0, 3) =
              low_cost_location::lidar_tag_poses.front().position;
          detected_tag_pose.block<3, 3>(0, 0) =
              low_cost_location::lidar_tag_poses.front()
                  .orientation.toRotationMatrix();

          // 计算初始姿态
          Eigen::Matrix4d init_frame_pose_mat =
              ref_lidar_tag_pose_.front() * detected_tag_pose.inverse();

          Eigen::Vector3d init_translation =
              init_frame_pose_mat.block<3, 1>(0, 3);
          Eigen::Quaterniond init_quater(init_frame_pose_mat.block<3, 3>(0, 0));
          multi_sensor_mapping::PoseData init_pose;
          init_pose.position = init_translation;
          init_pose.orientation = init_quater;

          locator_->InputInitialPose(init_pose);

          AINFO_F(
              "[APILowCostLocation] Set initial pose from topic [ ( %f, "
              "%f, %f ) (%f, %f, %f, %f)]",
              init_translation(0), init_translation(1), init_translation(2),
              init_quater.w(), init_quater.x(), init_quater.y(),
              init_quater.z());

          // 停止检测
          detector_->Stop();
        }

        low_cost_location::update_lidar_tag_flag = false;
      }
    }
    usleep(1000);
  }
}

void APILowCostLocation::VisualizationProcess() {
  int pix_x, pix_y;
  float pose_angle;
  bool upload_visualization_flag = true;
  // 地图可视化
  visualizer_->DisplayGlobalCloud(low_cost_location::global_cloud);

  while (!exit_process_flag_) {
    // 处理定位姿态
    {
      std::lock_guard<std::mutex> lg(low_cost_location::location_pose_mtx);
      if (low_cost_location::update_location_pose_flag) {
        // 上传3D姿态
        if (cb_location_pose_) {
          StampedPose pose;
          pose.position = low_cost_location::location_pose.position;
          pose.orientation = low_cost_location::location_pose.orientation;
          cb_location_pose_(pose);
        }
        // 上传2D姿态
        if (cb_location_2d_pose_) {
          Eigen::Vector3f pose_2d;
          pose_2d(0) = low_cost_location::location_pose.position(0);
          pose_2d(1) = low_cost_location::location_pose.position(1);
          double roll, pitch, yaw;
          multi_sensor_mapping::utils::RoationMatrixd2YPR(
              low_cost_location::location_pose.orientation.toRotationMatrix(),
              yaw, pitch, roll);
          pose_2d(2) = yaw / M_PI * 180.0 + low_cost_location::direction_offset;
          cb_location_2d_pose_(pose_2d);
        }
        // 上传像素姿态
        if (cb_location_pixel_pose_) {
          if (visualizer_->PoseToPixelCoordinate(
                  low_cost_location::location_pose.position,
                  low_cost_location::location_pose.orientation, pix_x, pix_y,
                  pose_angle)) {
            cb_location_pixel_pose_(Eigen::Vector3f(
                pix_x, pix_y,
                pose_angle + low_cost_location::direction_offset));
          }
        }
        // 更新
        upload_visualization_flag = visualizer_->RefreshGlobalMap(
            low_cost_location::location_pose.position);

        low_cost_location::update_location_pose_flag = false;
      }
    }

    // 上传可视化
    if (cb_visualization_img_) {
      if (upload_visualization_flag) {
        if (visualizer_->GetVisulizationImage(
                low_cost_location::visualization_img)) {
          cb_visualization_img_(low_cost_location::visualization_img);

          // 若地图更新，对应的点位都需要更新
          low_cost_location::refresh_mark_points_pose_flag = true;
          low_cost_location::refresh_highlight_mark_pose_flag = true;
        }

        upload_visualization_flag = false;
      }
    }

    // 上传标记点位信息
    if (cb_mark_points_pixel_pose_) {
      if (low_cost_location::refresh_mark_points_pose_flag) {
        if (!mark_pose_container_.empty()) {
          std::vector<Eigen::Vector2f> mark_points_pix_pose_vec;
          for (size_t i = 0; i < mark_pose_container_.size(); i++) {
            int mark_point_pix_x, mark_point_pix_y;
            if (visualizer_->PoseToPixelCoordinate(mark_pose_container_[i],
                                                   mark_point_pix_x,
                                                   mark_point_pix_y)) {
              mark_points_pix_pose_vec.push_back(
                  Eigen::Vector2f(mark_point_pix_x, mark_point_pix_y));
            }
          }

          cb_mark_points_pixel_pose_(mark_points_pix_pose_vec);
        }

        low_cost_location::refresh_mark_points_pose_flag = false;
      }
    }

    // 上传重点标记点位信息
    if (cb_highlight_mark_point_pixel_pose_) {
      if (low_cost_location::refresh_highlight_mark_pose_flag &&
          low_cost_location::highlight_mark_pose(2) > 0) {
        int mark_point_pix_x, mark_point_pix_y;
        if (visualizer_->PoseToPixelCoordinate(
                low_cost_location::highlight_mark_pose, mark_point_pix_x,
                mark_point_pix_y)) {
          cb_highlight_mark_point_pixel_pose_(
              Eigen::Vector2f(mark_point_pix_x, mark_point_pix_y));
        }

        low_cost_location::refresh_highlight_mark_pose_flag = false;
      }
    }

    usleep(1000);
  }
}

void APILowCostLocation::Reset() {
  low_cost_location::ResetGloablParam();

  tag_init_done_flag_ = false;
  ref_lidar_tag_pose_.clear();
  mark_pose_container_.clear();

  visualizer_ =
      std::make_shared<multi_sensor_mapping::LidarMapper2DVisualizer>();
}
