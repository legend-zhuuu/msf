#include "multi_sensor_mapping/core/odometry_manager.h"

#include "multi_sensor_mapping/core/fast_lio_mapper.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

namespace multi_sensor_mapping {

OdometryManager::OdometryManager()
    : exit_process_flag_(false), init_done_flag_(false), start_flag_(false) {}

void OdometryManager::SetParamSetPath(std::string _param_path) {
  param_set_path_ = _param_path;
}

void OdometryManager::Init() {
  // Step1 加载参数集合
  std::shared_ptr<ParamSet> param_set(new ParamSet);
  if (!param_set->Load(param_set_path_)) {
    return;
  }

  param_set->PrintAll();

  // Step2 初始化里程计
  lidar_mapper_ = std::make_shared<FastLIOMapper>();
  lidar_mapper_->SetParams(param_set);
  lidar_mapper_->Init();

  // Step3 注册建图器回调函数
  lidar_mapper_->RegLidarPoseCallback(
      std::bind(&OdometryManager::PutLidarPose, this, std::placeholders::_1));

  lidar_mapper_->RegFrameCallback(std::bind(&OdometryManager::PutFrame, this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));

  // Step4 初始化可视化器
  visualizer_ = std::make_shared<LidarMapperRosVisualizer>(param_set);

  init_done_flag_ = true;
}

void OdometryManager::Start() {
  if (lidar_mapper_->IsRunning()) {
    AINFO_F(
        "[OdometryManager] LidarMapper process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[OdometryManager] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  // 交互线程
  interactive_thread_ =
      std::thread(std::bind(&OdometryManager::InteractiveProcess, this));

  // 开启建图线程
  lidar_mapper_->Start();

  start_flag_ = true;
}

void OdometryManager::Stop() {
  if (!start_flag_) {
    return;
  }

  if (lidar_mapper_->IsRunning()) {
    lidar_mapper_->Stop();
  }

  exit_process_flag_ = true;

  interactive_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;

  AINFO_F("[OdometryManager] OdometryManager process has been closed");
}

void OdometryManager::InteractiveProcess() {
  while (!exit_process_flag_ && ros::ok()) {
    if (!lidar_pose_queue_.Empty()) {
      PublishLidarPose(lidar_pose_queue_.Pop());
    }

    if (!frame_queue_.Empty()) {
      auto frame_info = frame_queue_.Pop();
      PublishFrame(frame_info.first, frame_info.second);
    }

    ros::spinOnce();
    usleep(10);
  }
}

void OdometryManager::PutLidarPose(const PoseData& _pose_data) {
  lidar_pose_queue_.Push(_pose_data);
}

void OdometryManager::PutFrame(const CloudTypePtr& _cloud,
                               const PoseData& _pose) {
  static int jump_cnt = 0;
  if (jump_cnt % 10 == 0) {
    frame_queue_.Push(std::make_pair(_cloud, _pose));
  }
  jump_cnt++;
}

void OdometryManager::PublishLidarPose(PoseData _pose_data) {
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
  pose_matrix.block<3, 1>(0, 3) = _pose_data.position;
  pose_matrix.block<3, 3>(0, 0) = _pose_data.orientation.toRotationMatrix();

  // 发布姿态
  visualizer_->DisplayCurrentPose(pose_matrix);
  visualizer_->DisplayCurrentOdom(_pose_data.timestamp, pose_matrix);
}

void OdometryManager::PublishFrame(CloudTypePtr _cloud, PoseData _pose) {
  CloudTypePtr cloud_in_map_frame(new CloudType());
  pcl::transformPointCloud(*_cloud, *cloud_in_map_frame, _pose.position,
                           _pose.orientation);
  visualizer_->DisplayCurrentCloud(cloud_in_map_frame);
}

}  // namespace multi_sensor_mapping