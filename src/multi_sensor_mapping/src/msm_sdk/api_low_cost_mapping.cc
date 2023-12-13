#include "msm_sdk/api_low_cost_mapping.h"

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "multi_sensor_mapping/core/fast_lio_mapper.h"
#include "multi_sensor_mapping/core/lidar_detector.h"
#include "multi_sensor_mapping/core/tiny_pose_graph_processor.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/param/lidar_detection_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/tiny_location_params.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_2d_visualizer.h"

using msm_sdk::APILowCostMapping;

namespace low_cost_mapping {
/// @brief 实时建图标志位
bool mapping_online_flag = true;
/// @brief 关键帧阈值标志位
double key_pose_threshold;
/// @brief 关键帧更新标志位
bool keyframe_update_flag = false;
/// @brief tag初始化标志位
bool tag_init_done_flag = false;
/// @brief 更新tag像素坐标标志位
bool update_tag_pixel_pose_flag = false;

/// @brief 使能激光检测标志位
bool enable_detection_flag = false;

/// @brief 关键帧点云
CloudTypePtr key_frame_cloud(new CloudType);
/// @brief 关键帧位姿
Eigen::Vector3d key_frame_position;
Eigen::Quaterniond key_frame_rotaion;

/// @brief 后端优化进度
double backend_progress = 0;

/// @brief 锁
std::mutex lio_pose_mtx;
std::mutex lio_frame_mtx;
std::mutex progress_mtx;
std::mutex lidar_tag_mtx;

// 缓存空间
std::vector<multi_sensor_mapping::PoseData> lio_pose_queue;
std::vector<std::pair<CloudType, multi_sensor_mapping::PoseData>>
    lio_frame_queue;
std::vector<CloudType> global_cloud_queue;
std::vector<CloudType> key_frame_pose_cloud_queue;
/// @brief 激光tag姿态
std::deque<std::vector<multi_sensor_mapping::PoseData>> lidar_tag_queue;
/// @brief 激光位姿缓存
std::deque<multi_sensor_mapping::PoseData> lidar_pose_cache;

/// @brief tag姿态
std::vector<multi_sensor_mapping::PoseData> lidar_tag_pose_container;

/// @brief 方向角偏置
float direction_offset = 180;

// 可视化图像
cv::Mat visualization_img;

void PutLioPose(const multi_sensor_mapping::PoseData& _pose) {
  std::lock_guard<std::mutex> lg(lio_pose_mtx);
  lio_pose_queue.push_back(_pose);
  if (enable_detection_flag && !tag_init_done_flag) {
    lidar_pose_cache.push_back(_pose);
    if (lidar_pose_cache.size() > 20) {
      lidar_pose_cache.pop_front();
    }
  }
}

void PutLioFrame(const CloudTypePtr& _cloud,
                 const multi_sensor_mapping::PoseData& _pose) {
  std::lock_guard<std::mutex> lg(lio_frame_mtx);
  lio_frame_queue.push_back(std::make_pair(*_cloud, _pose));
}

void PutBackendProgress(const double& _progress) {
  std::lock_guard<std::mutex> lg(progress_mtx);
  backend_progress = _progress;
}

void PutGlobalCloud(const CloudTypePtr& _cloud) {
  global_cloud_queue.push_back(*_cloud);
}

void PutKeyframePoseCloud(const CloudTypePtr& _cloud) {
  key_frame_pose_cloud_queue.push_back(*_cloud);
}

void PutLidarTagPose(
    const std::vector<multi_sensor_mapping::PoseData>& _tag_poses) {
  std::lock_guard<std::mutex> lg(lidar_tag_mtx);
  if (!_tag_poses.empty()) {
    lidar_tag_queue.push_back(_tag_poses);
  }
}

bool CheckKeyScan(const Eigen::Vector3d& _position) {
  static Eigen::Vector3d last_key_frame_pose(0, 0, 0);

  double delta_pose_sqr = (_position(0) - last_key_frame_pose(0)) *
                              (_position(0) - last_key_frame_pose(0)) +
                          (_position(1) - last_key_frame_pose(1)) *
                              (_position(1) - last_key_frame_pose(1));

  if (delta_pose_sqr > key_pose_threshold) {
    last_key_frame_pose = _position;
    return true;
  }
  return false;
}

void SaveLidarTagPose(std::string _cache_path) {
  if (lidar_tag_pose_container.empty()) {
    return;
  }

  if (!boost::filesystem::exists(_cache_path)) {
    boost::filesystem::create_directories(_cache_path);
  }

  YAML::Node tag_info_node;
  for (auto pose : lidar_tag_pose_container) {
    YAML::Node tag_node;
    tag_node["x"] = pose.position(0);
    tag_node["y"] = pose.position(1);
    tag_node["z"] = pose.position(2);

    tag_node["qx"] = pose.orientation.x();
    tag_node["qy"] = pose.orientation.y();
    tag_node["qz"] = pose.orientation.z();
    tag_node["qw"] = pose.orientation.w();
    tag_info_node.push_back(tag_node);
  }

  std::string tag_info_path = _cache_path + "/tag_info.yaml";
  std::ofstream tag_info_file(tag_info_path.c_str());
  tag_info_file << tag_info_node;
  tag_info_file.close();
}

void ResetGloablParam() {
  mapping_online_flag = true;
  keyframe_update_flag = false;
  tag_init_done_flag = false;
  update_tag_pixel_pose_flag = false;
  backend_progress = 0;

  lio_pose_queue.clear();
  lio_frame_queue.clear();
  global_cloud_queue.clear();
  key_frame_pose_cloud_queue.clear();
  lidar_tag_queue.clear();
  lidar_pose_cache.clear();
  lidar_tag_pose_container.clear();
}

}  // namespace low_cost_mapping

APILowCostMapping::APILowCostMapping()
    : exit_process_flag_(false), start_flag_(false), init_done_flag_(false) {}

void APILowCostMapping::SetMapCachePath(std::string _path) {
  map_cache_path_ = _path;
}

void APILowCostMapping::SetVisualizationResolution(float _resolution) {
  visualizer_->SetMapResolution(_resolution);
}

void APILowCostMapping::SetDirectionOffset(float _direction_offset) {
  low_cost_mapping::direction_offset = _direction_offset;
}

bool APILowCostMapping::Init(std::string _param_path) {
  // Step1 加载参数
  std::shared_ptr<multi_sensor_mapping::ParamSet> param_set(
      new multi_sensor_mapping::ParamSet);
  if (!param_set->Load(_param_path)) {
    return false;
  }

  low_cost_mapping::key_pose_threshold =
      param_set->GetFastLIOParams()->keyframe_adding_threshold *
      param_set->GetFastLIOParams()->keyframe_adding_threshold;

  low_cost_mapping::direction_offset =
      param_set->GetTinyLocationParams()->direction_offset;

  std::shared_ptr<multi_sensor_mapping::LidarDetectionParams>
      lidar_detection_params = param_set->GetLidarDetectionParams();
  lidar_detector_ = std::make_shared<multi_sensor_mapping::LidarDetector>();
  if (lidar_detection_params && lidar_detection_params->enable_detection) {
    low_cost_mapping::enable_detection_flag = true;
  }

  // Step2 初始化建图器
  mapper_ = std::make_shared<multi_sensor_mapping::FastLIOMapper>();
  mapper_->SetParams(param_set);
  mapper_->Init();

  // Step3 注册建图器回调函数
  mapper_->RegLidarPoseCallback(
      std::bind(&low_cost_mapping::PutLioPose, std::placeholders::_1));

  mapper_->RegFrameCallback(std::bind(&low_cost_mapping::PutLioFrame,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

  // Step4 初始化后端处理器
  pose_graph_processor_ =
      std::make_shared<multi_sensor_mapping::TinyPoseGraphProcessor>();
  pose_graph_processor_->SetParams(param_set);
  pose_graph_processor_->Init();

  // Step5 注册后端处理器回调函数
  pose_graph_processor_->RegProcessCallback(
      std::bind(&low_cost_mapping::PutBackendProgress, std::placeholders::_1));
  pose_graph_processor_->RegGlobalCloudCallback(
      std::bind(&low_cost_mapping::PutGlobalCloud, std::placeholders::_1));
  pose_graph_processor_->RegKeyFramePosesCallback(std::bind(
      &low_cost_mapping::PutKeyframePoseCloud, std::placeholders::_1));

  // Step6 初始化激光检测器

  lidar_detector_->SetParams(param_set);
  if (low_cost_mapping::enable_detection_flag) {
    lidar_detector_->Init();
    lidar_detector_->RegTagPoses(
        std::bind(&low_cost_mapping::PutLidarTagPose, std::placeholders::_1));
  }

  // Step7 初始化地图
  lidar_session_ = std::make_shared<multi_sensor_mapping::LidarMapSession>();

  // Step8 初始化可视化器
  visualizer_ =
      std::make_shared<multi_sensor_mapping::LidarMapper2DVisualizer>();

  init_done_flag_ = true;

  return true;
}

void APILowCostMapping::Start() {
  if (start_flag_) {
    AINFO_F("[APILowCostMapping] Mapping process thread have already running");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[APILowCostMapping] Please init system before start");
    return;
  }
  exit_process_flag_ = false;

  lidar_mapping_thread_ =
      std::thread(std::bind(&APILowCostMapping::MappingProcess, this));
  visualization_thread_ =
      std::thread(std::bind(&APILowCostMapping::VisualizationProcess, this));

  mapper_->Start();

  if (low_cost_mapping::enable_detection_flag) {
    lidar_detector_->Start();
  }

  low_cost_mapping::mapping_online_flag = true;
  start_flag_ = true;
}

void APILowCostMapping::StartBackend() {
  AINFO_F("[LowCostMapperManager] Start Mapping");

  // 关闭在线里程计线程
  if (mapper_->IsRunning()) {
    mapper_->Stop();
  }

  // 开启后端优化线程
  pose_graph_processor_->SetBaseMapSession(lidar_session_);
  pose_graph_processor_->SetCachePath(map_cache_path_);
  pose_graph_processor_->Start();

  low_cost_mapping::mapping_online_flag = false;
}

void APILowCostMapping::Stop() {
  if (!start_flag_) {
    return;
  }

  if (mapper_->IsRunning()) {
    mapper_->Stop();
  }

  if (pose_graph_processor_->IsRunning()) {
    pose_graph_processor_->Stop();
  }

  if (lidar_detector_->IsRunning()) {
    lidar_detector_->Stop();
  }

  exit_process_flag_ = true;

  lidar_mapping_thread_.join();
  AINFO_F("[APILowCostMapping] Mapping process thread joined");
  visualization_thread_.join();
  AINFO_F("[APILowCostMapping] Visualization process thread joined");

  start_flag_ = false;

  Reset();

  AINFO_F("[APILowCostMapping] Low cost Mapping process thread stopped");
}

void APILowCostMapping::RegVisualizationImgCallback(
    const std::function<void(const cv::Mat&)>& _cb_img) {
  cb_visualization_img_ = _cb_img;
}

void APILowCostMapping::RegProgressCallback(
    const std::function<void(const double&)>& _cb_progress) {
  cb_progress_ = _cb_progress;
}

void APILowCostMapping::RegLocationPoseCallback(
    const std::function<void(const StampedPose&)>& _cb_location_pose) {
  cb_location_pose_ = _cb_location_pose;
}

void APILowCostMapping::RegLocation2dPoseCallback(
    const std::function<void(const Eigen::Vector3f&)>& _cb_location_2d_pose) {
  cb_location_2d_pose_ = _cb_location_2d_pose;
}

void APILowCostMapping::RegLocationPixPoseCallback(
    const std::function<void(const Eigen::Vector3f&)>&
        _cb_location_pixel_pose) {
  cb_location_pixel_pose_ = _cb_location_pixel_pose;
}

void APILowCostMapping::RegSensorAbnormalCallback(
    const std::function<void(const bool&)>& _cb_sensor_abnormal) {
  cb_sensor_abnormal_ = _cb_sensor_abnormal;
}

void APILowCostMapping::RegKeyFramePixelPoseCallback(
    const std::function<void(const std::vector<Eigen::Vector2f>&)>&
        _cb_key_frame_pixel_pose) {
  cb_key_frame_pixel_pose_ = _cb_key_frame_pixel_pose;
}

void APILowCostMapping::RegTagPixelPoseCallback(
    const std::function<void(const Eigen::Vector3f&)>& _cb_tag_pixel_pose) {
  cb_tag_pixel_pose_ = _cb_tag_pixel_pose;
}

void APILowCostMapping::MappingProcess() {
  while (!exit_process_flag_) {
    if (low_cost_mapping::mapping_online_flag) {
      // 处理激光数据
      std::lock_guard<std::mutex> lg(low_cost_mapping::lio_frame_mtx);
      if (!low_cost_mapping::lio_frame_queue.empty()) {
        auto frame_info = low_cost_mapping::lio_frame_queue.back();
        KeyframeSelection(frame_info.second.timestamp, frame_info.first,
                          frame_info.second.position,
                          frame_info.second.orientation);

        low_cost_mapping::lio_frame_queue.clear();
      }

      // 处理激光tag姿态
      if (!low_cost_mapping::lidar_tag_queue.empty()) {
        low_cost_mapping::lidar_tag_mtx.lock();
        auto lidar_tag_queue_copy = low_cost_mapping::lidar_tag_queue;
        low_cost_mapping::lidar_tag_mtx.unlock();

        low_cost_mapping::lio_pose_mtx.lock();
        auto lidar_pose_cache_copy = low_cost_mapping::lidar_pose_cache;
        low_cost_mapping::lio_pose_mtx.unlock();

        for (int i = lidar_tag_queue_copy.size() - 1; i >= 0; i--) {
          double frame_timestamp = lidar_tag_queue_copy[i].front().timestamp;
          bool pose_found_flag = false;
          multi_sensor_mapping::PoseData frame_pose;
          // 查找对应的姿态
          for (int j = lidar_pose_cache_copy.size() - 1; j >= 0; j--) {
            if (fabs(lidar_pose_cache_copy[j].timestamp - frame_timestamp) <
                0.005) {
              frame_pose = lidar_pose_cache_copy[j];
              pose_found_flag = true;
              // AINFO_F("< %.6f , %.6f >", frame_timestamp,
              //         lidar_pose_cache_copy[j].timestamp);
              break;
            }
          }

          if (pose_found_flag) {
            low_cost_mapping::lidar_tag_pose_container.resize(
                lidar_tag_queue_copy[i].size());
            for (size_t k = 0; k < lidar_tag_queue_copy[i].size(); k++) {
              low_cost_mapping::lidar_tag_pose_container[k].orientation =
                  frame_pose.orientation *
                  lidar_tag_queue_copy[i][k].orientation;
              low_cost_mapping::lidar_tag_pose_container[k].position =
                  frame_pose.orientation * lidar_tag_queue_copy[i][k].position +
                  frame_pose.position;
            }
            low_cost_mapping::update_tag_pixel_pose_flag = true;
            low_cost_mapping::tag_init_done_flag = true;
            break;
          }
        }

        if (low_cost_mapping::tag_init_done_flag) {
          lidar_detector_->Stop();
          low_cost_mapping::lidar_tag_queue.clear();
          AINFO_F(
              "[LowCostMapperManager] Lidar Tag initialization success, Stop "
              "lidar detector");
        }
      }

      if (mapper_->SensorDisconnected()) {
        if (cb_sensor_abnormal_) {
          cb_sensor_abnormal_(true);
        }
        usleep(1000000);
      }
    } else {
      // 处理进度数据
      if (cb_progress_) {
        cb_progress_(low_cost_mapping::backend_progress);
        usleep(100000);
      }

      AINFO_F("[APILowCostMapping] backend progress < %f >",
              low_cost_mapping::backend_progress);

      // 建图完成，更新地图和保存地图
      if (low_cost_mapping::backend_progress == 1.0) {
        // 若tag初始化完成
        if (low_cost_mapping::tag_init_done_flag) {
          // 根据tag计算地图旋转角度
          Eigen::Quaterniond tag_orientation =
              low_cost_mapping::lidar_tag_pose_container.front().orientation;
          Eigen::Quaterniond rotation_y = Eigen::Quaterniond(
              Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d(0, 1, 0)));
          tag_orientation *= rotation_y;
          double yaw, pitch, roll;
          multi_sensor_mapping::utils::RoationMatrixd2YPR(
              tag_orientation.toRotationMatrix(), yaw, pitch, roll);
          // 为了保证初始化在上面，减去90°偏置
          Eigen::Quaterniond map_rotation = Eigen::Quaterniond(
              Eigen::AngleAxisd(-yaw - M_PI / 2, Eigen::Vector3d(0, 0, 1)));

          // 旋转地图
          pose_graph_processor_->SetMapRotation(map_rotation);

          // 旋转tag
          for (size_t i = 0;
               i < low_cost_mapping::lidar_tag_pose_container.size(); i++) {
            low_cost_mapping::lidar_tag_pose_container[i].position =
                map_rotation *
                low_cost_mapping::lidar_tag_pose_container[i].position;
            low_cost_mapping::lidar_tag_pose_container[i].orientation =
                map_rotation *
                low_cost_mapping::lidar_tag_pose_container[i].orientation;
          }

          usleep(1000000);
        } else {
          // 保存地图
          low_cost_mapping::SaveLidarTagPose(map_cache_path_);
          AINFO_F("[APILowCostMapping] Save map");
          pose_graph_processor_->SaveMap();
        }
      } else if (low_cost_mapping::backend_progress == 2.0) {
        // 保存地图
        low_cost_mapping::SaveLidarTagPose(map_cache_path_);
        pose_graph_processor_->SaveMap();
      }
    }

    usleep(10000);
  }
  AINFO_F("[APILowCostMapping] Mapping process thread stopped");
}

void APILowCostMapping::VisualizationProcess() {
  int pix_x, pix_y;
  float pose_angle;
  while (!exit_process_flag_) {
    {
      // 定位姿态可视化
      std::lock_guard<std::mutex> lg(low_cost_mapping::lio_pose_mtx);
      if (!low_cost_mapping::lio_pose_queue.empty()) {
        auto latest_pose = low_cost_mapping::lio_pose_queue.back();

        // 上传3D姿态
        if (cb_location_pose_) {
          StampedPose pose;
          pose.position = latest_pose.position;
          pose.orientation = latest_pose.orientation;
          cb_location_pose_(pose);
        }
        // 上传2D姿态
        if (cb_location_2d_pose_) {
          Eigen::Vector3f pose_2d;
          pose_2d(0) = latest_pose.position(0);
          pose_2d(1) = latest_pose.position(1);
          double roll, pitch, yaw;
          multi_sensor_mapping::utils::RoationMatrixd2YPR(
              latest_pose.orientation.toRotationMatrix(), yaw, pitch, roll);
          pose_2d(1) = yaw / M_PI * 180.0 + low_cost_mapping::direction_offset;
          cb_location_2d_pose_(pose_2d);
        }
        // 上传像素姿态
        if (cb_location_pixel_pose_) {
          if (visualizer_->PoseToPixelCoordinate(latest_pose.position,
                                                 latest_pose.orientation, pix_x,
                                                 pix_y, pose_angle)) {
            cb_location_pixel_pose_(Eigen::Vector3f(
                pix_x, pix_y, pose_angle + low_cost_mapping::direction_offset));
          }
        }
        low_cost_mapping::lio_pose_queue.clear();
      }
    }

    if (cb_visualization_img_) {
      if (low_cost_mapping::keyframe_update_flag) {
        visualizer_->DisplayCloud(low_cost_mapping::key_frame_cloud,
                                  low_cost_mapping::key_frame_position,
                                  low_cost_mapping::key_frame_rotaion);

        if (visualizer_->GetVisulizationImage(
                low_cost_mapping::visualization_img)) {
          cb_visualization_img_(low_cost_mapping::visualization_img);

          // 更新tag像素坐标
          if (low_cost_mapping::tag_init_done_flag) {
            low_cost_mapping::update_tag_pixel_pose_flag = true;
          }
        }

        low_cost_mapping::keyframe_update_flag = false;
      }

      // 全局地图显示
      if (!low_cost_mapping::global_cloud_queue.empty()) {
        visualizer_->DisplayGlobalCloud(
            low_cost_mapping::global_cloud_queue.front().makeShared());
        low_cost_mapping::global_cloud_queue.clear();
        if (visualizer_->GetVisulizationImage(
                low_cost_mapping::visualization_img)) {
          cb_visualization_img_(low_cost_mapping::visualization_img);

          // 更新tag像素坐标
          if (low_cost_mapping::tag_init_done_flag) {
            low_cost_mapping::update_tag_pixel_pose_flag = true;
          }
        }
      }
    }

    // 上传关键帧位姿
    if (cb_key_frame_pixel_pose_) {
      if (!low_cost_mapping::key_frame_pose_cloud_queue.empty()) {
        std::vector<Eigen::Vector2f> key_frame_pixel_pose;
        CloudType keyframe_cloud =
            low_cost_mapping::key_frame_pose_cloud_queue.front();
        low_cost_mapping::key_frame_pose_cloud_queue.clear();
        for (const PointType& p : keyframe_cloud.points) {
          int pix_x, pix_y;
          if (visualizer_->PoseToPixelCoordinate(Eigen::Vector3d(p.x, p.y, 0),
                                                 pix_x, pix_y)) {
            key_frame_pixel_pose.push_back(Eigen::Vector2f(pix_x, pix_y));
          }
        }
        cb_key_frame_pixel_pose_(key_frame_pixel_pose);
      }
    }

    // 上传tag像素位姿
    if (cb_tag_pixel_pose_) {
      if (low_cost_mapping::update_tag_pixel_pose_flag) {
        int pix_x, pix_y;
        Eigen::Vector3d tag_pose =
            low_cost_mapping::lidar_tag_pose_container.front().position;
        if (visualizer_->PoseToPixelCoordinate(
                Eigen::Vector3d(tag_pose(0), tag_pose(1), 0), pix_x, pix_y)) {
          cb_tag_pixel_pose_(Eigen::Vector3f(pix_x, pix_y, 0));
        }
        low_cost_mapping::update_tag_pixel_pose_flag = false;
      }
    }

    usleep(10000);
  }

  AINFO_F("[APILowCostMapping] Visualization process thread stopped");
}

void APILowCostMapping::KeyframeSelection(double _timestamp,
                                          const CloudType& _cloud,
                                          const Eigen::Vector3d& _position,
                                          const Eigen::Quaterniond& _rotation) {
  if (_cloud.size() < 1000) {
    return;
  }

  // 判断关键帧
  if (lidar_session_->GetMapUnitSize() == 0 ||
      low_cost_mapping::CheckKeyScan(_position)) {
    *low_cost_mapping::key_frame_cloud = _cloud;

    // 添加关键帧
    Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
    pose_mat.block<3, 3>(0, 0) = _rotation.toRotationMatrix();
    pose_mat.block<3, 1>(0, 3) = _position;
    std::shared_ptr<multi_sensor_mapping::LidarMapUnit> map_unit(
        new multi_sensor_mapping::LidarMapUnit(
            _timestamp, low_cost_mapping::key_frame_cloud, pose_mat));
    lidar_session_->AddMapUnit(map_unit);

    low_cost_mapping::key_frame_position = _position;
    low_cost_mapping::key_frame_rotaion = _rotation;

    low_cost_mapping::keyframe_update_flag = true;

    // AINFO_F("[APILowCostMapping] Add key frame, pose: %f, %f, %f",
    // _position(0),
    //         _position(1), _position(2));
  }
}

void APILowCostMapping::Reset() {
  low_cost_mapping::ResetGloablParam();
  AINFO_F("[APILowCostMapping] Reset Params Done");

  lidar_session_ = std::make_shared<multi_sensor_mapping::LidarMapSession>();

  visualizer_ =
      std::make_shared<multi_sensor_mapping::LidarMapper2DVisualizer>();
}