#include "multi_sensor_mapping/core/tiny_lidar_locator.h"

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/frontend/lidar/ndt_scan_matcher.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/tiny_location_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

TinyLidarLocator::TinyLidarLocator()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      location_status_(LS_IDLE),
      update_map_flag_(false),
      update_init_pose_flag_(false),
      update_key_frame_flag_(false),
      update_key_frame_pose_flag_(false),
      update_lio_pose_flag_(false),
      map_cloud_(new CloudType),
      latest_key_frame_cloud_(new CloudType) {
  key_frame_pose_bias_ = Eigen::Matrix4d::Identity();
}

void TinyLidarLocator::SetParams(const std::shared_ptr<ParamSet>& _param_set) {
  location_params_ = _param_set->GetTinyLocationParams();
}

void TinyLidarLocator::SetMapPath(std::string _map_path) {
  map_cloud_->clear();

  std::string map_cloud_path = _map_path;
  boost::filesystem::path map_path(_map_path);
  if (map_path.extension() != ".pcd") {
    map_cloud_path = _map_path + "/global_cloud.pcd";
  }

  if (pcl::io::loadPCDFile(map_cloud_path, *map_cloud_) == -1) {
    AWARN_F("[TinyLidarLocator] Map Cloud Path is wrong : %s",
            map_cloud_path.c_str());
    return;
  }

  update_map_flag_ = true;
}

void TinyLidarLocator::SetMapCloud(const CloudTypePtr& _map_cloud) {
  map_cloud_ = _map_cloud;
  update_map_flag_ = true;
}

void TinyLidarLocator::Init() {
  AINFO_F("[TinyLidarLocator] Init Tiny-Lidar-Locator");

  if (!location_params_) {
    AWARN_F("[TinyLidarLocator] No input Params");
    return;
  }

  // 初始化激光配准器
  scan_matcher_ = std::make_shared<NdtScanMatcher>(
      location_params_->target_resolution, location_params_->source_resolution,
      location_params_->num_cores);

  init_done_flag_ = true;
}

void TinyLidarLocator::Start() {
  if (!init_done_flag_) {
    AWARN_F("[TinyLidarLocator] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  global_location_thread_ =
      std::thread(std::bind(&TinyLidarLocator::GlobalLocationProcess, this));
  fusion_thread_ =
      std::thread(std::bind(&TinyLidarLocator::FusionProcess, this));

  start_flag_ = true;
}

void TinyLidarLocator::Stop() {
  if (!start_flag_) {
    return;
  }
  exit_process_flag_ = true;

  global_location_thread_.join();
  fusion_thread_.join();

  start_flag_ = false;

  ResetLocator();

  AINFO_F("[TinyLidarLocator] Tiny Lidar locator process thread stopped");
}

void TinyLidarLocator::InputInitialPose(const PoseData& _init_pose) {
  initial_pose_ = _init_pose;
  update_init_pose_flag_ = true;

  AINFO_F("[TinyLidarLocator] Receive Initial pose < %f, %f, %f >",
          _init_pose.position(0), _init_pose.position(1),
          _init_pose.position(2));
}

void TinyLidarLocator::InputLIOPose(const PoseData& _pose) {
  {
    Lock lock_lio(lio_pose_mutex_);
    laetst_lio_pose_ = _pose;
  }
  update_lio_pose_flag_ = true;
}

void TinyLidarLocator::InputKeyFrame(const CloudTypePtr& _cloud,
                                     const PoseData& _pose) {
  *latest_key_frame_cloud_ = *_cloud;
  latest_key_frame_pose_ = _pose;

  update_key_frame_flag_ = true;
}

void TinyLidarLocator::RegLocationPose(
    const std::function<void(const PoseData&)>& _cb_location_pose) {
  cb_put_location_pose_ = _cb_location_pose;
}

void TinyLidarLocator::RegLocationCloud(
    const std::function<void(const CloudTypePtr&)>& _cb_location_cloud) {
  cb_put_location_cloud_ = _cb_location_cloud;
}

void TinyLidarLocator::GlobalLocationProcess() {
  while (!exit_process_flag_) {
    if (location_status_ == LS_IDLE) {
      // 空闲状态等待地图
      if (update_map_flag_) {
        // 初始化NDT
        scan_matcher_->SetTargetCloud(map_cloud_);

        update_map_flag_ = false;

        location_status_ = LS_INITIALIZING;

        AINFO_F("[TinyLidarLocator] IDLE --> INITIALIZING");
      }
    } else if (location_status_ == LS_INITIALIZING) {
      if (update_init_pose_flag_ && update_key_frame_flag_) {
        Eigen::Matrix4f initial_pose_mat = Eigen::Matrix4f::Identity();
        initial_pose_mat.block<3, 3>(0, 0) =
            initial_pose_.orientation.toRotationMatrix().template cast<float>();
        initial_pose_mat.block<3, 1>(0, 3) =
            initial_pose_.position.template cast<float>();

        // NDT 姿态估计
        Eigen::Matrix4f pose_estimated;
        scan_matcher_->Match(latest_key_frame_cloud_, initial_pose_mat,
                             pose_estimated);

        // 更新关键帧偏置
        Eigen::Matrix4d key_frame_lio_pose = Eigen::Matrix4d::Identity();
        key_frame_lio_pose.block<3, 3>(0, 0) =
            latest_key_frame_pose_.orientation.toRotationMatrix();
        key_frame_lio_pose.block<3, 1>(0, 3) = latest_key_frame_pose_.position;
        {
          Lock lock_key_frame_pose(key_frame_pose_mutex_);
          key_frame_pose_bias_ = pose_estimated.template cast<double>() *
                                 key_frame_lio_pose.inverse();
        }

        // 上传点云
        if (cb_put_location_cloud_) {
          CloudTypePtr cloud_in_map_frame(new CloudType);
          pcl::transformPointCloud(*latest_key_frame_cloud_,
                                   *cloud_in_map_frame, pose_estimated);
          cb_put_location_cloud_(cloud_in_map_frame);
        }

        update_key_frame_pose_flag_ = true;

        update_init_pose_flag_ = false;
        update_key_frame_flag_ = false;

        location_status_ = LS_NORMAL;
        AINFO_F("[TinyLidarLocator] INITIALIZING --> NORMAL");
      }
    } else if (location_status_ == LS_NORMAL) {
      if (update_key_frame_flag_) {
        Eigen::Matrix4f pose_predicted =
            OdometryPose2LocationPose(latest_key_frame_pose_)
                .template cast<float>();
        // NDT 姿态估计
        Eigen::Matrix4f pose_estimated;
        scan_matcher_->Match(latest_key_frame_cloud_, pose_predicted,
                             pose_estimated);

        // 更新关键帧偏置
        Eigen::Matrix4d key_frame_lio_pose = Eigen::Matrix4d::Identity();
        key_frame_lio_pose.block<3, 3>(0, 0) =
            latest_key_frame_pose_.orientation.toRotationMatrix();
        key_frame_lio_pose.block<3, 1>(0, 3) = latest_key_frame_pose_.position;
        {
          Lock lock_key_frame_pose(key_frame_pose_mutex_);
          key_frame_pose_bias_ = pose_estimated.template cast<double>() *
                                 key_frame_lio_pose.inverse();
        }

        update_key_frame_pose_flag_ = true;

        // 上传点云
        if (cb_put_location_cloud_) {
          CloudTypePtr cloud_in_map_frame(new CloudType);
          pcl::transformPointCloud(*latest_key_frame_cloud_,
                                   *cloud_in_map_frame, pose_estimated);
          cb_put_location_cloud_(cloud_in_map_frame);
        }

        update_key_frame_flag_ = false;
      }
    } else if (location_status_ == LS_ERROR) {
      /// TOOD
    }

    usleep(10000);
  }
}

void TinyLidarLocator::FusionProcess() {
  while (!exit_process_flag_) {
    if (location_status_ == LS_NORMAL) {
      if (update_lio_pose_flag_) {
        Eigen::Matrix4d location_pose_mat;
        {
          Lock lock_lio(lio_pose_mutex_);
          location_pose_mat = OdometryPose2LocationPose(laetst_lio_pose_);
        }

        // 上传姿态
        if (cb_put_location_pose_) {
          PoseData location_pose;
          location_pose.timestamp = laetst_lio_pose_.timestamp;
          location_pose.position = location_pose_mat.block<3, 1>(0, 3);
          location_pose.orientation =
              Eigen::Quaterniond(location_pose_mat.block<3, 3>(0, 0));
          cb_put_location_pose_(location_pose);
        }

        update_lio_pose_flag_ = false;
      }
    }

    usleep(1000);
  }
}

Eigen::Matrix4d TinyLidarLocator::OdometryPose2LocationPose(
    const PoseData& _odom_pose_data) {
  Eigen::Matrix4d odom_pose_mat = Eigen::Matrix4d::Identity();
  odom_pose_mat.block<3, 1>(0, 3) = _odom_pose_data.position;
  odom_pose_mat.block<3, 3>(0, 0) =
      _odom_pose_data.orientation.toRotationMatrix();

  return key_frame_pose_bias_ * odom_pose_mat;
}

bool TinyLidarLocator::IsRunning() { return start_flag_; }

void TinyLidarLocator::ResetLocator() {
  location_status_ = LS_IDLE;
  update_map_flag_ = false;
  update_init_pose_flag_ = false;
  update_key_frame_flag_ = false;
  update_key_frame_pose_flag_ = false;
  update_lio_pose_flag_ = false;

  map_cloud_ = CloudTypePtr(new CloudType);
  latest_key_frame_cloud_ = CloudTypePtr(new CloudType);
  key_frame_pose_bias_ = Eigen::Matrix4d::Identity();
}

}  // namespace multi_sensor_mapping