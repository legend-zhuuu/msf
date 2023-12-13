#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "msm_sdk/api_lidar_imu_mapping.h"
#include "multi_sensor_mapping/core/lidar_imu_mapper.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"

using msm_sdk::APILidarImuMapping;

std::mutex lidar_pose_mtx;
std::mutex frame_cloud_mtx;
std::mutex progress_mtx;

// 缓存空间
std::vector<multi_sensor_mapping::PoseData> lidar_pose_queue;
std::vector<std::pair<CloudType, multi_sensor_mapping::PoseData> >
    frame_cloud_queue;
double mapping_progress = 0;
std::shared_ptr<multi_sensor_mapping::LidarMapSession> active_map_session;

msm_sdk::MappingScene mapping_scene = msm_sdk::MappingScene::SCENE_DAFAULT;
bool use_loop_closure = false;

void PutMapperException(const multi_sensor_mapping::MSMError& _error_msg) {
  /// TODO
}

void PutLidarPose(const multi_sensor_mapping::PoseData& _pose) {
  std::lock_guard<std::mutex> lg(lidar_pose_mtx);
  lidar_pose_queue.push_back(_pose);
}

void PutFrame(const CloudTypePtr& _cloud,
              const multi_sensor_mapping::PoseData& _pose) {
  static int jump_cnt = 0;
  if (jump_cnt % 20 == 0) {
    std::lock_guard<std::mutex> lg(frame_cloud_mtx);
    frame_cloud_queue.push_back(std::make_pair(*_cloud, _pose));
  }
  jump_cnt++;
}

void PutKeyFrame(const CloudTypePtr& _cloud,
                 const multi_sensor_mapping::PoseData& _pose) {
  /// TODO
}

void PutMappingProgress(const double& _progress) {
  std::lock_guard<std::mutex> lg(progress_mtx);
  mapping_progress = _progress;
}

void PutMapSession(
    const std::shared_ptr<multi_sensor_mapping::LidarMapSession>& _session) {
  active_map_session = _session;
}

APILidarImuMapping::APILidarImuMapping()
    : exit_process_flag_(false), start_flag_(false), init_done_flag_(false) {}

void APILidarImuMapping::SetMappingParam(MappingScene _scene,
                                         bool _use_loop_closure) {
  mapping_scene = _scene;
  use_loop_closure = _use_loop_closure;
  AINFO_F("[APILidarImuMapping] Mapping scene : %i ", (int)mapping_scene);
}

bool APILidarImuMapping::Init(std::string _param_path) {
  // Step1 加载参数
  std::shared_ptr<multi_sensor_mapping::ParamSet> param_set(
      new multi_sensor_mapping::ParamSet);
  if (!param_set->Load(_param_path)) {
    return false;
  }

  // 参数修改
  if (mapping_scene != msm_sdk::MappingScene::SCENE_DAFAULT) {
    if (mapping_scene == msm_sdk::MappingScene::SCENE_INDOOR) {
      param_set->GetLidarImuMappingParams()->corner_ds_leaf_size = 0.1;
      param_set->GetLidarImuMappingParams()->surface_ds_leaf_size = 0.2;
    } else if (mapping_scene == msm_sdk::MappingScene::SCENE_OUTDOOR) {
      param_set->GetLidarImuMappingParams()->corner_ds_leaf_size = 0.2;
      param_set->GetLidarImuMappingParams()->surface_ds_leaf_size = 0.4;
    }
    param_set->GetLidarImuMappingParams()->auto_loop_closure = use_loop_closure;
  }

  // Step2 初始化建图器
  mapper_ = std::make_shared<multi_sensor_mapping::LidarImuMapper>();
  mapper_->SetParams(param_set);
  mapper_->Init();

  /// Step3 注册回调函数
  mapper_->RegExceptionCallback(
      std::bind(&PutMapperException, std::placeholders::_1));
  mapper_->RegLidarPoseCallback(
      std::bind(&PutLidarPose, std::placeholders::_1));
  mapper_->RegKeyFrameCallback(
      std::bind(&PutKeyFrame, std::placeholders::_1, std::placeholders::_2));
  mapper_->RegFrameCallback(
      std::bind(&PutFrame, std::placeholders::_1, std::placeholders::_2));
  mapper_->RegPutMappingProgress(
      std::bind(&PutMappingProgress, std::placeholders::_1));
  mapper_->RegPutMapSession(std::bind(&PutMapSession, std::placeholders::_1));

  init_done_flag_ = true;

  return true;
}

void APILidarImuMapping::Start(std::string _bag_path) {
  if (start_flag_) {
    AINFO_F("[APILidarImuMapping] Mapping process thread have already running");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[APILidarImuMapping] Please init system before start");
    return;
  }
  exit_process_flag_ = false;

  mapper_->SetBagPath(_bag_path);

  process_thead_ = std::thread(std::bind(&APILidarImuMapping::Process, this));

  mapper_->Start();

  start_flag_ = true;
}

void APILidarImuMapping::Stop() {
  if (!start_flag_) {
    return;
  }
  mapper_->Stop();
  exit_process_flag_ = true;
  process_thead_.join();

  mapping_progress = 0;

  start_flag_ = false;
}

void APILidarImuMapping::SaveMapSession(std::string _save_path) {
  std::string session_save_path;
  if (!active_map_session) {
    AWARN_F(
        "[APILidarImuMapping] Save map session failed, map session is null "
        "pointer");
    return;
  }
  if (_save_path.empty()) {
    session_save_path = active_map_session->GetCachePath();
  } else {
    session_save_path = _save_path;
  }

  active_map_session->Save(session_save_path);
}

std::shared_ptr<multi_sensor_mapping::LidarMapSession>
APILidarImuMapping::GetLidarMapSession() {
  return active_map_session;
}

void APILidarImuMapping::RegLidarPoseCallback(
    const std::function<void(const StampedPose&)>& _cb_pose) {
  cb_lidar_pose_ = _cb_pose;
}

void APILidarImuMapping::RegFrameCloudCallback(
    const std::function<void(const StampedCloud&)>& _cb_cloud) {
  cb_frame_cloud_ = _cb_cloud;
}

void APILidarImuMapping::RegMappingProgressCallback(
    const std::function<void(const double)>& _cb_progress) {
  cb_progress_ = _cb_progress;
}

void APILidarImuMapping::Process() {
  while (!exit_process_flag_) {
    {
      // 定位姿态处理
      std::lock_guard<std::mutex> lg(lidar_pose_mtx);
      if (!lidar_pose_queue.empty()) {
        if (cb_lidar_pose_) {
          StampedPose pose;
          pose.timestamp = lidar_pose_queue.back().timestamp;
          pose.position = lidar_pose_queue.back().position;
          pose.orientation = lidar_pose_queue.back().orientation;
          cb_lidar_pose_(pose);
        }
        lidar_pose_queue.clear();
      }
    }

    {
      // 处理激光点云
      std::lock_guard<std::mutex> lg(frame_cloud_mtx);
      if (!frame_cloud_queue.empty()) {
        if (cb_frame_cloud_) {
          auto frame_info = frame_cloud_queue.back();
          StampedCloud frame_cloud_in_map;
          pcl::transformPointCloud(frame_info.first, frame_cloud_in_map.cloud,
                                   frame_info.second.position,
                                   frame_info.second.orientation);
          cb_frame_cloud_(frame_cloud_in_map);
        }
        frame_cloud_queue.clear();
      }
    }

    {
      std::lock_guard<std::mutex> lg(progress_mtx);
      if (cb_progress_) {
        cb_progress_(mapping_progress);
      }
    }

    usleep(100000);
  }
}