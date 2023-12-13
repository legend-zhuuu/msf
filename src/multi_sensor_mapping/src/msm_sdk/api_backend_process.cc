#include "msm_sdk/api_backend_process.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "multi_sensor_mapping/core/pose_graph_processor.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_common.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

using msm_sdk::APIBackendProcess;

std::mutex pose_graph_mtx;
std::mutex global_cloud_mtx;
std::mutex local_match_mtx;
std::mutex process_state_mtx;

std::vector<multi_sensor_mapping::PoseGraph> pose_graph_cache_;
std::vector<CloudType> global_cloud_cache_;
std::vector<multi_sensor_mapping::LocalMatchInformation>
    local_match_info_cache_;
std::queue<ProcessState> process_state_cache_;

msm_sdk::MappingScene backend_scene = msm_sdk::MappingScene::SCENE_DAFAULT;

void PutPoseGraphException(const multi_sensor_mapping::MSMError& _error_msg) {
  /// TODO
}

void PutPoseGraph(const multi_sensor_mapping::PoseGraph& _graph) {
  std::lock_guard<std::mutex> lg(pose_graph_mtx);
  // AINFO_F("Put pose graph");
  pose_graph_cache_.push_back(_graph);
}

void PutGlobalCloud(const CloudTypePtr& _cloud) {
  std::lock_guard<std::mutex> lg(global_cloud_mtx);
  // AINFO_F("Put cloud");
  global_cloud_cache_.push_back(*_cloud);
}

void PutLocalMatchInfomation(
    const multi_sensor_mapping::LocalMatchInformation& _local_match_info) {
  std::lock_guard<std::mutex> lg(local_match_mtx);
  local_match_info_cache_.push_back(_local_match_info);
}

void PutProcessState(const ProcessState& _state) {
  std::lock_guard<std::mutex> lg(process_state_mtx);
  process_state_cache_.push(_state);
}

bool ConvertPoseGraph(const multi_sensor_mapping::PoseGraph& _graph,
                      msm_sdk::Graph& _out_graph) {
  _out_graph.vertices.clear();
  _out_graph.loop_closure_edges.clear();

  size_t vertic_size = _graph.vertices.size();
  _out_graph.vertices.reserve(vertic_size);
  for (size_t i = 0; i < vertic_size; i++) {
    _out_graph.vertices.push_back(_graph.vertices[i].position);
  }

  size_t lc_edge_size = _graph.loop_closure_edegs_.size();
  if (lc_edge_size > 0) {
    _out_graph.loop_closure_edges.reserve(lc_edge_size);
    for (size_t i = 0; i < lc_edge_size; i++) {
      msm_sdk::LoopClosureEdge lc_edge;
      lc_edge.id = (int)i;
      lc_edge.front_id = _graph.loop_closure_edegs_[i].front_id;
      lc_edge.back_id = _graph.loop_closure_edegs_[i].back_id;
      lc_edge.reg_score = _graph.loop_closure_edegs_[i].reg_score;
      lc_edge.valid_flag = _graph.loop_closure_edegs_[i].valid_flag;
      _out_graph.loop_closure_edges.push_back(lc_edge);
    }
  }

  // TODO GNSS edge
  return true;
}

bool ConvertLocalMatch(
    const multi_sensor_mapping::LocalMatchInformation& _local_match,
    msm_sdk::LoopClosureInfo& _lc_info) {
  _lc_info.loop_closure_id = _local_match.loop_closure_id;
  _lc_info.target_pos = _local_match.target_pose_in_map.block<3, 1>(0, 3);
  _lc_info.source_pos = _local_match.source_pose_in_map.block<3, 1>(0, 3);
  _lc_info.score = _local_match.score;

  Eigen::Matrix4d target_pose_in_local = _local_match.target_pose_in_map;
  target_pose_in_local.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix4d source_pose_in_local =
      target_pose_in_local * _local_match.relative_pose;
  CloudTypePtr target_cloud_in_local(new CloudType);
  pcl::transformPointCloud(_local_match.target_cloud, *target_cloud_in_local,
                           target_pose_in_local);

  CloudTypePtr source_cloud_in_local(new CloudType);
  pcl::transformPointCloud(_local_match.source_cloud, *source_cloud_in_local,
                           source_pose_in_local);

  multi_sensor_mapping::utils::DownsampleCloud(*target_cloud_in_local, 0.1);
  multi_sensor_mapping::utils::DownsampleCloud(*source_cloud_in_local, 0.1);

  _lc_info.target_cloud = *target_cloud_in_local;
  _lc_info.source_cloud = *source_cloud_in_local;

  double dx, dy, dz, dyaw, dpitch, droll;
  multi_sensor_mapping::utils::Transd2XYZYPR(source_pose_in_local, dx, dy, dz,
                                             dyaw, dpitch, droll);
  if (dyaw > 180) {
    dyaw -= 360;
  }
  _lc_info.relative_pos = Eigen::Vector3d(dx, dy, dz);
  _lc_info.relative_euler = Eigen::Vector3d(droll, dpitch, dyaw);

  return true;
}

APIBackendProcess::APIBackendProcess()
    : exit_process_flag_(false), start_flag_(false), init_done_flag_(false) {}

bool APIBackendProcess::Init(std::string _param_path) {
  // Step1 加载参数
  std::shared_ptr<multi_sensor_mapping::ParamSet> param_set(
      new multi_sensor_mapping::ParamSet);
  if (!param_set->Load(_param_path)) {
    return false;
  }

  // 参数修改
  if (backend_scene != msm_sdk::MappingScene::SCENE_DAFAULT) {
    if (backend_scene == msm_sdk::MappingScene::SCENE_INDOOR) {
      param_set->GetLidarImuMappingParams()->loop_closure_searching_distance =
          2.0;
      param_set->GetLidarImuMappingParams()->mono_layer_motion = false;
    } else if (backend_scene == msm_sdk::MappingScene::SCENE_OUTDOOR) {
      param_set->GetLidarImuMappingParams()->loop_closure_searching_distance =
          15.0;
      param_set->GetLidarImuMappingParams()->mono_layer_motion = true;
    }
  }

  // Step2 初始化后端处理器
  backend_ = std::make_shared<multi_sensor_mapping::PoseGraphProcessor>();
  backend_->SetParams(param_set);
  backend_->Init();

  // Step3 注册回调函数
  backend_->RegExceptionCallback(
      std::bind(&PutPoseGraphException, std::placeholders::_1));
  backend_->RegPoseGraphInfoCallback(
      std::bind(&PutPoseGraph, std::placeholders::_1));
  backend_->RegGlobalCloudCallback(
      std::bind(&PutGlobalCloud, std::placeholders::_1));
  backend_->RegLocalMatchInfoCallback(
      std::bind(&PutLocalMatchInfomation, std::placeholders::_1));
  backend_->RegProcessState(std::bind(&PutProcessState, std::placeholders::_1));

  init_done_flag_ = true;

  return true;
}

void APIBackendProcess::SetMappingParam(MappingScene _scene,
                                        bool _use_loop_closure) {
  backend_scene = _scene;
  AINFO_F("[APIBackendProcess] Mapping scene : %i ", (int)backend_scene);
}

void APIBackendProcess::Start(std::string _session_path) {
  if (start_flag_) {
    AINFO_F("[APIBackendProcess] Backend process thread have already running");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[APIBackendProcess] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  backend_->SetBaseMapSession(_session_path);

  process_thead_ = std::thread(std::bind(&APIBackendProcess::Process, this));

  backend_->Start();

  start_flag_ = true;
}

void APIBackendProcess::Start(
    const std::shared_ptr<multi_sensor_mapping::LidarMapSession>& _session) {
  if (start_flag_) {
    AINFO_F("[APIBackendProcess] Backend process thread have already running");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[APIBackendProcess] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  backend_->SetBaseMapSession(_session);

  process_thead_ = std::thread(std::bind(&APIBackendProcess::Process, this));

  backend_->Start();

  start_flag_ = true;
}

void APIBackendProcess::Stop() {
  if (!start_flag_) {
    return;
  }

  backend_->Stop();

  exit_process_flag_ = true;
  process_thead_.join();

  start_flag_ = false;
}

void APIBackendProcess::StartAutoLoopClosureDetection() {
  if (!backend_) {
    AWARN_F(
        "[APIBackendProcess] Please init system before Start Auto Loop Closure "
        "Detection");
    return;
  }

  backend_->StartAutoLoopClosureDetection();
}

void APIBackendProcess::StartBackendOptimization() {
  if (!backend_) {
    AWARN_F(
        "[APIBackendProcess] Please init system before Start Backend "
        "Optimization");
    return;
  }

  backend_->StartBackendOptimization();
}

void APIBackendProcess::StartManualLoopClosureDetection(int _lc_id) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->StartManualLoopClosureDetection(_lc_id);
}

void APIBackendProcess::AdjustLocalMatch(int _lc_id, double _x, double _y,
                                         double _z, double _roll, double _pitch,
                                         double _yaw) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->AdjustLocalMatch(_lc_id, _x, _y, _z, _roll, _pitch, _yaw);
}

void APIBackendProcess::AdjustMapResolution(double _resolution) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->AdjustMapResolution(_resolution);
}

void APIBackendProcess::MatchLocalMatch(int _lc_id, double _x, double _y,
                                        double _z, double _roll, double _pitch,
                                        double _yaw) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->MatchLocalMatch(_lc_id, _x, _y, _z, _roll, _pitch, _yaw);
}

void APIBackendProcess::UpdateLocalMatch(int _lc_id) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->UpdateLocalMatch(_lc_id);
}

void APIBackendProcess::StartPostProcess() {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->StartPostProcess();
}

void APIBackendProcess::UpdateMapSession() {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->UpdateMapSession();
}

void APIBackendProcess::AdjustDeltaSliceParam(double _delta_radius,
                                              double _delta_height,
                                              double _delta_thickness) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->AdjustSliceParam(_delta_radius, _delta_height, _delta_thickness);
}

void APIBackendProcess::AdjustSliceParam(double _radius, double _height,
                                         double _thickness) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->AdjustSliceParam2(_radius, _height, _thickness);
}

void APIBackendProcess::AdjustMapYaw(double _delta_yaw) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->AdjustMapYaw(_delta_yaw);
}

void APIBackendProcess::SaveLidarMapSession(std::string _save_path) {
  if (!backend_) {
    AWARN_F("[APIBackendProcess] Please init system before Save Map Session ");
    return;
  }

  backend_->SaveLidarMapSession(_save_path);
}

void APIBackendProcess::RegGraphCallback(
    const std::function<void(const Graph&)>& _cb_graph) {
  cb_graph_ = _cb_graph;
}

void APIBackendProcess::RegLoopClosureCallback(
    const std::function<void(const LoopClosureInfo&)>& _cb_loop_closure) {
  cb_loop_closure_ = _cb_loop_closure;
}

void APIBackendProcess::RegGlobalMapCallback(
    const std::function<void(const StampedCloud&)>& _cb_global_map) {
  cb_global_cloud_ = _cb_global_map;
}

void APIBackendProcess::RegProcessStateCallback(
    const std::function<void(const bool&)>& _cb_process_state) {
  cb_process_state_ = _cb_process_state;
}

void APIBackendProcess::Process() {
  while (!exit_process_flag_) {
    {
      // 处理graph
      std::lock_guard<std::mutex> lg(pose_graph_mtx);
      if (!pose_graph_cache_.empty()) {
        if (cb_graph_) {
          Graph graph;
          multi_sensor_mapping::PoseGraph latest_graph =
              pose_graph_cache_.back();

          ConvertPoseGraph(latest_graph, graph);
          cb_graph_(graph);
        }
        pose_graph_cache_.clear();
      }
    }

    {
      // 处理 Global Cloud
      std::lock_guard<std::mutex> lg(global_cloud_mtx);
      if (!global_cloud_cache_.empty()) {
        if (cb_global_cloud_) {
          auto cloud = global_cloud_cache_.back();
          StampedCloud global_cloud;
          pcl::copyPointCloud(cloud, global_cloud.cloud);
          cb_global_cloud_(global_cloud);
        }
        global_cloud_cache_.clear();
      }
    }

    {
      // 处理 LoopClosure Info
      std::lock_guard<std::mutex> lg(local_match_mtx);
      if (!local_match_info_cache_.empty()) {
        if (cb_loop_closure_) {
          auto latest_local_match = local_match_info_cache_.back();
          LoopClosureInfo lc_info;
          ConvertLocalMatch(latest_local_match, lc_info);
          cb_loop_closure_(lc_info);
        }
        local_match_info_cache_.clear();
      }
    }

    {
      // 处理process state
      std::lock_guard<std::mutex> lg(process_state_mtx);
      if (!process_state_cache_.empty()) {
        auto state = process_state_cache_.front();
        process_state_cache_.pop();
        if (cb_process_state_) {
          if (state == ProcessState::PROCESS_FINSIH) {
            cb_process_state_(true);
          }
        }
      }
    }

    usleep(100000);
  }
}