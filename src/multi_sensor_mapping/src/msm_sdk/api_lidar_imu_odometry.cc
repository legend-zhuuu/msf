#include "msm_sdk/api_lidar_imu_odometry.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "multi_sensor_mapping/core/fast_lio_mapper.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/utils/utils_log.h"

using msm_sdk::APILidarImuOdometry;

std::mutex lio_pose_mtx;
std::mutex lio_frame_mtx;

// 缓存空间
std::vector<multi_sensor_mapping::PoseData> lio_pose_queue;
std::vector<std::pair<CloudType, multi_sensor_mapping::PoseData> >
    lio_frame_queue;

void PutLioPose(const multi_sensor_mapping::PoseData& _pose) {
  std::lock_guard<std::mutex> lg(lio_pose_mtx);
  lio_pose_queue.push_back(_pose);
}

void PutLioFrame(const CloudTypePtr& _cloud,
                 const multi_sensor_mapping::PoseData& _pose) {
  static int jump_cnt = 0;
  if (jump_cnt % 2 == 0) {
    std::lock_guard<std::mutex> lg(lio_frame_mtx);
    lio_frame_queue.push_back(std::make_pair(*_cloud, _pose));
  }
  jump_cnt++;
}

APILidarImuOdometry::APILidarImuOdometry()
    : exit_process_flag_(false), start_flag_(false), init_done_flag_(false) {}

bool APILidarImuOdometry::Init(std::string _param_path) {
  // Step1 加载参数
  std::shared_ptr<multi_sensor_mapping::ParamSet> param_set(
      new multi_sensor_mapping::ParamSet);
  if (!param_set->Load(_param_path)) {
    return false;
  }

  // Step2 初始化建图器
  mapper_ = std::make_shared<multi_sensor_mapping::FastLIOMapper>();
  mapper_->SetParams(param_set);
  mapper_->Init();

  // Step3 注册建图器回调函数
  mapper_->RegLidarPoseCallback(std::bind(&PutLioPose, std::placeholders::_1));

  mapper_->RegFrameCallback(
      std::bind(&PutLioFrame, std::placeholders::_1, std::placeholders::_2));

  init_done_flag_ = true;

  return true;
}

void APILidarImuOdometry::Start() {
  if (start_flag_) {
    AINFO_F(
        "[APILidarImuOdometry] Odometry process thread have already running");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[APILidarImuOdometry] Please init system before start");
    return;
  }
  exit_process_flag_ = false;

  process_thead_ = std::thread(std::bind(&APILidarImuOdometry::Process, this));

  mapper_->Start();

  start_flag_ = true;
}

void APILidarImuOdometry::Stop() {
  if (!start_flag_) {
    return;
  }
  mapper_->Stop();
  exit_process_flag_ = true;
  process_thead_.join();

  start_flag_ = false;
}

void APILidarImuOdometry::RegLidarPoseCallback(
    const std::function<void(const StampedPose&)>& _cb_pose) {
  cb_lidar_pose_ = _cb_pose;
}

void APILidarImuOdometry::RegFrameCloudCallback(
    const std::function<void(const StampedCloud&)>& _cb_cloud) {
  cb_frame_cloud_ = _cb_cloud;
}

void APILidarImuOdometry::Process() {
  while (!exit_process_flag_) {
    {
      // 定位姿态处理
      std::lock_guard<std::mutex> lg(lio_pose_mtx);
      if (!lio_pose_queue.empty()) {
        if (cb_lidar_pose_) {
          StampedPose pose;
          pose.timestamp = lio_pose_queue.back().timestamp;
          pose.position = lio_pose_queue.back().position;
          pose.orientation = lio_pose_queue.back().orientation;
          cb_lidar_pose_(pose);
        }
        lio_pose_queue.clear();
      }
    }

    {
      // 处理激光点云
      std::lock_guard<std::mutex> lg(lio_frame_mtx);
      if (!lio_frame_queue.empty()) {
        if (cb_frame_cloud_) {
          auto frame_info = lio_frame_queue.back();
          StampedCloud frame_cloud_in_map;
          pcl::transformPointCloud(frame_info.first, frame_cloud_in_map.cloud,
                                   frame_info.second.position,
                                   frame_info.second.orientation);
          cb_frame_cloud_(frame_cloud_in_map);
        }
        lio_frame_queue.clear();
      }
    }

    usleep(100000);
  }
}