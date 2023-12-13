#include "multi_sensor_mapping/core/mapper_manager.h"

#include "multi_sensor_mapping/backend/loop_closure_detector.h"
#include "multi_sensor_mapping/core/clins_mapper.h"
#include "multi_sensor_mapping/core/lidar_imu_mapper.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

namespace multi_sensor_mapping {

MapperManager::MapperManager()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      lidar_map_session_update_flag_(false) {}

void MapperManager::SetParamSetPath(std::string _param_path) {
  param_set_path_ = _param_path;
}

void MapperManager::SetBagPath(std::string _bag_path, double _start_time,
                               double _map_durr) {
  bag_path_ = _bag_path;
  bag_start_time_ = _start_time;
  bag_durr_ = _map_durr;
}

void MapperManager::SetSessionPath(std::string _session_path) {
  if (pose_graph_processor_) {
    pose_graph_processor_->SetBaseMapSession(_session_path);
  } else {
    AWARN_F(
        "[MapperManager] Please init pose_graph_processor before set session "
        "path. ");
  }
}

void MapperManager::StartAutoLoopClosureDetection() {
  pose_graph_processor_->StartAutoLoopClosureDetection();
}

void MapperManager::Init() {
  // Step1 加载参数集合
  std::shared_ptr<ParamSet> param_set(new ParamSet);
  if (!param_set->Load(param_set_path_)) {
    return;
  }

  mapping_mode_ = param_set->mapping_mode_;

  RigisterSub();

  // Step2 初始化建图器
  if (mapping_mode_ == MappingMode::LIDAR_IMU_MAPPING) {
    lidar_mapper_ = std::make_shared<LidarImuMapper>();
    lidar_mapper_->SetParams(param_set);
    lidar_mapper_->Init();
  } else if (mapping_mode_ == MappingMode::LIDAR_ODOM_MAPPING) {
    /// TODO
  } else if (mapping_mode_ == MappingMode::CLINS_MAPPING) {
    lidar_mapper_ = std::make_shared<ClinsMapper>();
    lidar_mapper_->SetParams(param_set);
    lidar_mapper_->Init();
  }

  // Step3 注册建图器回调函数
  lidar_mapper_->RegExceptionCallback(std::bind(
      &MapperManager::PutMapperException, this, std::placeholders::_1));
  lidar_mapper_->RegLidarPoseCallback(
      std::bind(&MapperManager::PutLidarPose, this, std::placeholders::_1));
  lidar_mapper_->RegKeyFrameCallback(std::bind(&MapperManager::PutKeyFrame,
                                               this, std::placeholders::_1,
                                               std::placeholders::_2));
  lidar_mapper_->RegFrameCallback(std::bind(&MapperManager::PutFrame, this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
  lidar_mapper_->RegPutMappingProgress(std::bind(
      &MapperManager::PutMappingProgress, this, std::placeholders::_1));
  lidar_mapper_->RegPutMapSession(
      std::bind(&MapperManager::PutMapSession, this, std::placeholders::_1));

  // Step4 初始化后端处理器
  pose_graph_processor_ = std::make_shared<PoseGraphProcessor>();
  pose_graph_processor_->SetParams(param_set);
  pose_graph_processor_->Init();

  // Step5 注册后端优化器回调函数
  pose_graph_processor_->RegExceptionCallback(std::bind(
      &MapperManager::PutPoseGraphException, this, std::placeholders::_1));
  pose_graph_processor_->RegPoseGraphInfoCallback(
      std::bind(&MapperManager::PutPoseGraph, this, std::placeholders::_1));
  pose_graph_processor_->RegGlobalCloudCallback(
      std::bind(&MapperManager::PutGlobalCloud, this, std::placeholders::_1));
  pose_graph_processor_->RegLocalMatchInfoCallback(std::bind(
      &MapperManager::PutLocalMatchInfomation, this, std::placeholders::_1));

  // Step6 初始化可视化器
  visualizer_ = std::make_shared<LidarMapperRosVisualizer>(param_set);

  init_done_flag_ = true;
}

void MapperManager::Start() {
  if (lidar_mapper_->IsRunning()) {
    AINFO_F("[MapperManager] LidarMapper process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[MapperManager] Please init system before start");
    return;
  }

  lidar_mapper_->SetBagPath(bag_path_);
  lidar_mapper_->SetMappingPeriod(bag_start_time_, bag_durr_);

  exit_process_flag_ = false;

  // 交互线程
  interactive_thread_ =
      std::thread(std::bind(&MapperManager::InteractiveProcess, this));

  // 地图渲染线程
  visualization_thread_ = std::thread(
      std::bind(&MapperManager::GlobalMapVisualizationProcess, this));

  // 开启建图线程
  lidar_mapper_->Start();
  // 开始后端优化线程
  pose_graph_processor_->Start();

  start_flag_ = true;
}

void MapperManager::StartFrontend() {
  if (lidar_mapper_->IsRunning()) {
    AINFO_F("[MapperManager] LidarMapper process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[MapperManager] Please init system before start");
    return;
  }

  lidar_mapper_->SetBagPath(bag_path_);

  exit_process_flag_ = false;

  // 交互线程
  interactive_thread_ =
      std::thread(std::bind(&MapperManager::InteractiveProcess, this));

  // 地图渲染线程
  visualization_thread_ = std::thread(
      std::bind(&MapperManager::GlobalMapVisualizationProcess, this));

  lidar_mapper_->Start();

  start_flag_ = true;
}

void MapperManager::StartBackend() {
  if (pose_graph_processor_->IsRunning()) {
    AINFO_F(
        "[MapperManager] Pose-Graph-Processor process thread have already "
        "running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[MapperManager] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  // 交互线程
  interactive_thread_ =
      std::thread(std::bind(&MapperManager::InteractiveProcess, this));
  // 地图渲染线程
  visualization_thread_ = std::thread(
      std::bind(&MapperManager::GlobalMapVisualizationProcess, this));

  pose_graph_processor_->Start();

  start_flag_ = true;
}

void MapperManager::Stop() {
  if (!start_flag_) {
    return;
  }

  if (lidar_mapper_->IsRunning()) {
    lidar_mapper_->Stop();
  }

  if (pose_graph_processor_->IsRunning()) {
    pose_graph_processor_->Stop();
  }

  exit_process_flag_ = true;

  interactive_thread_.join();
  visualization_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;

  AINFO_F("[MapperManager] MapperManager process has been closed");
}

void MapperManager::SaveActiveMapSession() {
  if (!active_session_) {
    AWARN_F("[MapperManager] Save map failed, map session is empty");
  }

  active_session_->Save(active_session_->GetCachePath());
  AINFO_F("[MapperManager] Save map session success ");
}

void MapperManager::RigisterSub() {
  sub_backend_panel_cmd_ =
      nh_.subscribe<multi_sensor_mapping::backend_panel_cmd>(
          "/backend_panel_cmd", 10, &MapperManager::BackendPanelCmdHandler,
          this);
  pub_backend_info_ = nh_.advertise<multi_sensor_mapping::backend_panel_cmd>(
      "/backend_info", 10);
}

void MapperManager::BackendPanelCmdHandler(
    const multi_sensor_mapping::backend_panel_cmd::ConstPtr& _panel_cmd) {
  if (_panel_cmd->cmd == "ADJ") {
    pose_graph_processor_->AdjustLocalMatch(
        int(_panel_cmd->id), _panel_cmd->x, _panel_cmd->y, _panel_cmd->z,
        _panel_cmd->roll, _panel_cmd->pitch, _panel_cmd->yaw);
  } else if (_panel_cmd->cmd == "MATCH") {
    pose_graph_processor_->MatchLocalMatch(
        int(_panel_cmd->id), _panel_cmd->x, _panel_cmd->y, _panel_cmd->z,
        _panel_cmd->roll, _panel_cmd->pitch, _panel_cmd->yaw);
  } else if (_panel_cmd->cmd == "CONFIRM") {
    pose_graph_processor_->UpdateLocalMatch(int(_panel_cmd->id));
  } else if (_panel_cmd->cmd == "AUTOLCD") {
    pose_graph_processor_->StartAutoLoopClosureDetection();
  } else if (_panel_cmd->cmd == "OPT") {
    pose_graph_processor_->StartBackendOptimization();
  } else if (_panel_cmd->cmd == "MANUALLCD") {
    pose_graph_processor_->StartManualLoopClosureDetection(int(_panel_cmd->id));
  } else if (_panel_cmd->cmd == "SAVE") {
    pose_graph_processor_->SaveLidarMapSession();
  } else if (_panel_cmd->cmd == "POST") {
    pose_graph_processor_->StartPostProcess();
  } else if (_panel_cmd->cmd == "SLICEADJ") {
    pose_graph_processor_->AdjustSliceParam(_panel_cmd->slice_radius,
                                            _panel_cmd->slice_height,
                                            _panel_cmd->slice_thick);
  } else if (_panel_cmd->cmd == "MAPADJ") {
    pose_graph_processor_->AdjustMapYaw(_panel_cmd->yaw);
  } else if (_panel_cmd->cmd == "UPDATESESSION") {
    pose_graph_processor_->UpdateMapSession();
  }
}

void MapperManager::InteractiveProcess() {
  while (!exit_process_flag_ && ros::ok()) {
    if (!mapper_error_queue_.Empty()) {
      PublishErrorInfo(mapper_error_queue_.Pop());
    }

    if (!lidar_pose_queue_.Empty()) {
      PublishLidarPose(lidar_pose_queue_.Pop());
    }

    if (!frame_queue_.Empty()) {
      auto frame_info = frame_queue_.Pop();
      PublishFrame(frame_info.first, frame_info.second);
    }

    if (!mapping_progress_queue_.Empty()) {
      PublishMappingProgress(mapping_progress_queue_.Pop());
    }

    if (!pose_graph_queue_.Empty()) {
      PublishPoseGraph(pose_graph_queue_.Pop());
    }

    if (!local_match_info_queue_.Empty()) {
      LocalMatchInformation info = local_match_info_queue_.Pop();
      PublishLocalMatchInfo(info);
    }

    if (lidar_map_session_update_flag_) {
      lidar_map_session_update_flag_ = false;
      pose_graph_processor_->SetBaseMapSession(active_session_);
    }

    ros::spinOnce();
    usleep(10);
  }
}

void MapperManager::GlobalMapVisualizationProcess() {
  while (!exit_process_flag_ && ros::ok()) {
    if (!key_frame_queue_.Empty()) {
      auto key_frame_info = key_frame_queue_.Pop();
      /// TODO
    }

    if (!global_cloud_queue_.Empty()) {
      auto cloud = global_cloud_queue_.Pop();
      visualizer_->DisplayGlobalCloud(cloud);
    }

    usleep(10000);
  }
}

void MapperManager::PutMapperException(const MSMError& _error_msg) {
  // mapper_error_queue_.Push(_error_msg);
}

void MapperManager::PutLidarPose(const PoseData& _pose_data) {
  lidar_pose_queue_.Push(_pose_data);
}

void MapperManager::PutKeyFrame(const CloudTypePtr& _cloud,
                                const PoseData& _pose) {
  // key_frame_queue_.Push(std::make_pair(_cloud, _pose));

  // TODO
}

void MapperManager::PutFrame(const CloudTypePtr& _cloud,
                             const PoseData& _pose) {
  static int jump_cnt = 0;
  if (jump_cnt % 10 == 0) {
    frame_queue_.Push(std::make_pair(_cloud, _pose));
  }
  jump_cnt++;
}

void MapperManager::PutMappingProgress(const double& _progress) {
  mapping_progress_queue_.Push(_progress);
}

void MapperManager::PutPoseGraphException(const MSMError& _error_msg) {
  /// TODO
}

void MapperManager::PutPoseGraph(const PoseGraph& _graph) {
  pose_graph_queue_.Push(_graph);
}

void MapperManager::PutGlobalCloud(const CloudTypePtr& _cloud) {
  global_cloud_queue_.Push(_cloud);
}

void MapperManager::PutLocalMatchInfomation(
    const LocalMatchInformation& _local_match_info) {
  local_match_info_queue_.Push(_local_match_info);
}

void MapperManager::PutMapSession(
    const std::shared_ptr<LidarMapSession>& _session) {
  active_session_ = _session;
  lidar_map_session_update_flag_ = true;
}

void MapperManager::PublishErrorInfo(MSMError _error) {
  std::cout << _error.ToString() << std::endl;
}

void MapperManager::PublishLidarPose(PoseData _pose_data) {
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
  pose_matrix.block<3, 1>(0, 3) = _pose_data.position;
  pose_matrix.block<3, 3>(0, 0) = _pose_data.orientation.toRotationMatrix();

  // 发布姿态
  visualizer_->DisplayCurrentPose(pose_matrix);
  visualizer_->DisplayCurrentOdom(_pose_data.timestamp, pose_matrix);
}

void MapperManager::PublishFrame(CloudTypePtr _cloud, PoseData _pose) {
  CloudTypePtr cloud_in_map_frame(new CloudType());
  pcl::transformPointCloud(*_cloud, *cloud_in_map_frame, _pose.position,
                           _pose.orientation);
  visualizer_->DisplayCurrentCloud(cloud_in_map_frame);
}

void MapperManager::PublishKeyFrame(CloudTypePtr _cloud, PoseData _pose) {
  /// TODO
}

void MapperManager::PublishMappingProgress(double _progress) {}

void MapperManager::PublishPoseGraph(const PoseGraph& _graph) {
  visualizer_->DisplayPoseGraph(_graph);
}

void MapperManager::PublishLocalMatchInfo(
    const LocalMatchInformation& _local_match_info) {
  Eigen::Matrix4d target_pose_in_local = _local_match_info.target_pose_in_map;
  target_pose_in_local.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix4d source_pose_in_local =
      target_pose_in_local * _local_match_info.relative_pose;

  CloudTypePtr target_cloud_in_local(new CloudType);
  pcl::transformPointCloud(_local_match_info.target_cloud,
                           *target_cloud_in_local, target_pose_in_local);

  CloudTypePtr source_cloud_in_local(new CloudType);
  pcl::transformPointCloud(_local_match_info.source_cloud,
                           *source_cloud_in_local, source_pose_in_local);

  utils::DownsampleCloud(*target_cloud_in_local, 0.1);
  utils::DownsampleCloud(*source_cloud_in_local, 0.1);

  visualizer_->DisplayLoopClosureCloud(target_cloud_in_local,
                                       source_cloud_in_local);

  visualizer_->DisplaySelectedEdge(
      _local_match_info.source_pose_in_map.block<3, 1>(0, 3),
      _local_match_info.target_pose_in_map.block<3, 1>(0, 3));

  double dx, dy, dz, dyaw, dpitch, droll;
  utils::Transd2XYZYPR(source_pose_in_local, dx, dy, dz, dyaw, dpitch, droll);
  if (dyaw > 180) {
    dyaw -= 360;
  }
  multi_sensor_mapping::backend_panel_cmd backend_msg;
  backend_msg.cmd = "LOCAL";
  backend_msg.x = dx;
  backend_msg.y = dy;
  backend_msg.z = dz;
  backend_msg.roll = droll;
  backend_msg.pitch = dpitch;
  backend_msg.yaw = dyaw;
  backend_msg.id = _local_match_info.loop_closure_id;
  pub_backend_info_.publish(backend_msg);
}

}  // namespace multi_sensor_mapping