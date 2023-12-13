#include "multi_sensor_mapping/core/lidar_detector.h"

#include "multi_sensor_mapping/frontend/lidar/lidar_tag_detector.h"
#include "multi_sensor_mapping/param/lidar_detection_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

LidarDetector::LidarDetector()
    : exit_process_flag_(false), init_done_flag_(false), start_flag_(false) {}

void LidarDetector::SetParams(const std::shared_ptr<ParamSet>& _param_set) {
  sensor_params_ = _param_set->GetSensorParams();
  lidar_detection_params_ = _param_set->GetLidarDetectionParams();
}

void LidarDetector::Init() {
  AINFO_F("[FastLIOMapper] Init Lidar Detector");

  // 检测参数
  if (!lidar_detection_params_ || !sensor_params_) {
    AWARN_F("[LidarDetector] No input Params");
    return;
  }

  // 初始化Lidar Detector核心
  lidar_tag_detector_ = std::make_shared<LidarTagDetector>(
      lidar_detection_params_->num_points_for_plane,
      lidar_detection_params_->intensity_threshold,
      lidar_detection_params_->depth_threshold,
      lidar_detection_params_->tag_size, lidar_detection_params_->leaf_size,
      lidar_detection_params_->layer_size);

  data_converter_ = std::make_shared<DataConverter>();

  // 注册传感器订阅器
  RegisterSensorSub();

  init_done_flag_ = true;
}

void LidarDetector::Start() {
  if (start_flag_) {
    AINFO_F(
        "[LidarDetector] Lidar Detector process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[LidarDetector] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  // 主处理线程
  detection_thread_ = std::thread(std::bind(&LidarDetector::LoopProcess, this));

  start_flag_ = true;
}

void LidarDetector::Stop() {
  if (!start_flag_) {
    return;
  }

  exit_process_flag_ = true;
  detection_thread_.join();

  start_flag_ = false;

  ResetDetector();

  AINFO_F("[LidarDetector] Lidar Detector process thread stopped");
}

void LidarDetector::RegTagPoses(
    const std::function<void(const std::vector<PoseData>&)>& _tag_poses) {
  cb_tag_poses_ = _tag_poses;
}

void LidarDetector::LoopProcess() {
  while (!exit_process_flag_) {
    if (!cloud_buffer_.empty()) {
      TimedCloudData current_cloud_data;
      {
        Lock lock_lidar(lidar_mutex_);
        current_cloud_data = cloud_buffer_.back();
        cloud_buffer_.clear();
      }

      int tag_num = lidar_tag_detector_->DetectTag(current_cloud_data.cloud);
      if (tag_num == lidar_detection_params_->lidar_tag_num) {
        if (cb_tag_poses_) {
          auto tag_poses = lidar_tag_detector_->GetTagPose();
          std::vector<PoseData> result_tag_pose;
          for (auto tag_pose : tag_poses) {
            PoseData pose_data;
            pose_data.timestamp = current_cloud_data.timestamp;
            pose_data.position =
                Eigen::Vector3d(tag_pose.position(0), tag_pose.position(1),
                                tag_pose.position(2));
            pose_data.orientation = Eigen::Quaterniond(
                tag_pose.orientation.w(), tag_pose.orientation.x(),
                tag_pose.orientation.y(), tag_pose.orientation.z());

            result_tag_pose.push_back(pose_data);
          }

          cb_tag_poses_(result_tag_pose);
        }
      }
    }

    if (!ros::ok()) break;
    ros::spinOnce();
    usleep(20000);
  }
}

void LidarDetector::RegisterSensorSub() {
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

  std::string lidar_topic = sensor_params_->LIOLidarTopic();
  sub_cloud_ = nh_->subscribe<sensor_msgs::PointCloud2>(
      lidar_topic, 100, &LidarDetector::CloudHandler, this,
      ros::TransportHints().tcpNoDelay());
}

void LidarDetector::CloudHandler(
    const sensor_msgs::PointCloud2::ConstPtr& _cloud_msg) {
  if (!start_flag_) {
    return;
  }

  TimedCloudData cloud_data;
  if (sensor_params_->LIOLidarFormat() == LidarDataFormat::RSLIDAR) {
    cloud_data = data_converter_->ConvertRsCloudData(_cloud_msg);
  } else if (sensor_params_->LIOLidarFormat() ==
             LidarDataFormat::RSLIDAR_LEGACY) {
    cloud_data = data_converter_->ConvertLegacyRsCloudData(_cloud_msg);
  } else {
    AWARN_F("[FastLIOMapper] Unknow lidar format");
    return;
  }

  Lock lock_lidar(lidar_mutex_);
  cloud_buffer_.push_back(cloud_data);
}

bool LidarDetector::IsRunning() { return start_flag_; }

void LidarDetector::ResetDetector() { cloud_buffer_.clear(); }

}  // namespace multi_sensor_mapping
