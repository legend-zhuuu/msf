#include "multi_sensor_mapping/core/tiny_pose_graph_processor.h"

#include "multi_sensor_mapping/backend/loop_closure_detector.h"
#include "multi_sensor_mapping/backend/pose_graph_manager.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/pose_graph_params.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

TinyPoseGraphProcessor::TinyPoseGraphProcessor()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      state_(TPGS_IDLE),
      loop_closure_skip_frame_(20),
      map_global_rotation_(Eigen::Quaterniond::Identity()) {}

void TinyPoseGraphProcessor::SetParams(
    const std::shared_ptr<ParamSet>& _param_set) {
  pose_graph_params_ = _param_set->GetPoseGraphParams();
}

void TinyPoseGraphProcessor::Init() {
  AINFO_F("[TinyPoseGraphProcessor] Init Tiny-Pose-Graph-Processor");

  if (!pose_graph_params_) {
    AWARN_F("[TinyPoseGraphProcessor] No input Params");
    return;
  }

  loop_closure_detector_ = std::make_shared<LoopClosureDetector>(
      pose_graph_params_->loop_closure_searching_distance,
      pose_graph_params_->loop_closure_distance_interval,
      pose_graph_params_->loop_closure_score_threshold,
      pose_graph_params_->mono_layer_motion);

  pose_graph_manager_ = std::make_shared<PoseGraphManager>();

  init_done_flag_ = true;
}

void TinyPoseGraphProcessor::Start() {
  if (!lidar_session_) {
    AWARN_F("[TinyPoseGraphProcessor] No input Lidar-Session");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[TinyPoseGraphProcessor] Please init system before start");
    return;
  }

  process_thread_ =
      std::thread(std::bind(&TinyPoseGraphProcessor::BackendProcess, this));

  exit_process_flag_ = false;
  start_flag_ = true;
}

void TinyPoseGraphProcessor::Stop() {
  if (!start_flag_) {
    return;
  }

  exit_process_flag_ = true;
  process_thread_.join();

  start_flag_ = false;

  Reset();

  AINFO_F("[TinyPoseGraphProcessor] Tiny Pose Graph process thread stopped");
}

void TinyPoseGraphProcessor::SetBaseMapSession(
    const std::shared_ptr<LidarMapSession>& _session) {
  lidar_session_ = _session;
}

void TinyPoseGraphProcessor::SetCachePath(std::string _cache_path) {
  cache_path_ = _cache_path;
}

void TinyPoseGraphProcessor::SetMapRotation(
    const Eigen::Quaterniond& _rotation) {
  map_global_rotation_ = _rotation;
  state_ = TPGS_ADJUST;
}

void TinyPoseGraphProcessor::SaveMap() { state_ = TPGS_SAVE; }

void TinyPoseGraphProcessor::RegProcessCallback(
    const std::function<void(const double&)>& _cb_progress) {
  cb_put_progress_ = _cb_progress;
}

void TinyPoseGraphProcessor::RegPoseGraphInfoCallback(
    const std::function<void(const PoseGraph&)>& _cb_pose_graph) {
  cb_pose_graph_ = _cb_pose_graph;
}

void TinyPoseGraphProcessor::RegLocalMatchInfoCallback(
    const std::function<void(const LocalMatchInformation&)>& _cb_local_match) {
  cb_local_match_info_ = _cb_local_match;
}

void TinyPoseGraphProcessor::RegGlobalCloudCallback(
    const std::function<void(const CloudTypePtr&)>& _cb_global_cloud) {
  cb_global_cloud_ = _cb_global_cloud;
}

void TinyPoseGraphProcessor::RegKeyFramePosesCallback(
    const std::function<void(const CloudTypePtr&)>& _cb_keyframe_cloud) {
  cb_keyframe_cloud_ = _cb_keyframe_cloud;
}

void TinyPoseGraphProcessor::BackendProcess() {
  // 上传位姿图
  if (cb_pose_graph_) {
    PoseGraph graph = lidar_session_->ExportPoseGraph();
    cb_pose_graph_(graph);
    AINFO_F("[PoseGraphProcessor] Upload pose graph ");
  }

  // 增量式闭环
  loop_closure_detector_->SetLidarMapSession(lidar_session_);
  pose_graph_manager_->ImportGraphFromMapSession(lidar_session_);

  int num_map_unit = lidar_session_->GetMapUnitSize();

  for (int i = 0; i < num_map_unit; i++) {
    LoopClosureResult loop_closure_result =
        loop_closure_detector_->DetectLoopClosure(i);

    if (loop_closure_result.success_flag) {
      AINFO_F(
          "[TinyPoseGraphProcessor] Loop closure detect success : %i ---> %i  "
          ", "
          "score < %f >",
          loop_closure_result.current_scan_id_,
          loop_closure_result.history_scan_id_, loop_closure_result.score_);

      // 添加闭环边
      pose_graph_manager_->AddLoopClosureEdge(
          loop_closure_result.history_scan_id_,
          loop_closure_result.current_scan_id_,
          loop_closure_result.trans_history_to_current,
          loop_closure_result.score_);

      // 闭环优化
      pose_graph_manager_->IncrementalPoseGraphOptimization();
      pose_graph_manager_->UpdateLidarMapSession(lidar_session_);

      // 上传闭环后的位姿图
      if (cb_pose_graph_) {
        PoseGraph graph = lidar_session_->ExportPoseGraph();
        cb_pose_graph_(graph);
        AINFO_F("[PoseGraphProcessor] Upload pose graph ");
      }

      if (cb_local_match_info_) {
        int loop_closure_id = lidar_session_->pose_graph_database_ptr_
                                  ->loop_closure_info_id_vec_.back();
        LocalMatchInformation local_match_info;
        lidar_session_->GetLocalMatchInformation(loop_closure_id,
                                                 local_match_info);
        cb_local_match_info_(local_match_info);
      }

      // std::cout << "next frame " << std::endl;
      // std::getchar();

      // 跳帧闭环后跳帧
      i += loop_closure_skip_frame_;
    }

    // 上传进度
    if (i % 10 == 0) {
      double progress = static_cast<double>(i) / double(num_map_unit);
      RunPutProgress(progress);
    }

    if (exit_process_flag_) {
      break;
    }
  }

  AINFO_F("[TinyPoseGraphProcessor] Loop closure finish");

  RunPutProgress(1.0);
  state_ = TPGS_IDLE;

  // 循环
  while (!exit_process_flag_) {
    if (state_ == TPGS_ADJUST) {
      // 调整地图
      lidar_session_->UpdateMapYaw(map_global_rotation_);
      state_ = TPGS_IDLE;
      // 上传全局点云
      if (cb_global_cloud_) {
        CloudTypePtr global_cloud =
            lidar_session_->GetGlobalMap(pose_graph_params_->map_resolution);
        cb_global_cloud_(global_cloud);
      }

      // 上传关键帧姿态
      if (cb_keyframe_cloud_) {
        CloudTypePtr keyframe_cloud = lidar_session_->GetKeyFramePoseCloud();
        cb_keyframe_cloud_(keyframe_cloud);
      }

      RunPutProgress(2.0);
    } else if (state_ == TPGS_SAVE) {
      // 上传全局点云
      if (cb_global_cloud_) {
        CloudTypePtr global_cloud =
            lidar_session_->GetGlobalMap(pose_graph_params_->map_resolution);
        cb_global_cloud_(global_cloud);
      }

      // 上传关键帧姿态
      if (cb_keyframe_cloud_) {
        CloudTypePtr keyframe_cloud = lidar_session_->GetKeyFramePoseCloud();
        cb_keyframe_cloud_(keyframe_cloud);
      }

      lidar_session_->Save(cache_path_, pose_graph_params_->map_resolution,
                           pose_graph_params_->output_detail_info,
                           pose_graph_params_->output_grid_map);
      AINFO_F("[TinyPoseGraphProcessor] Save lidar session in < %s >",
              cache_path_.c_str());

      state_ = TPGS_IDLE;
      RunPutProgress(3.0);
    }

    usleep(100000);
  }
}

void TinyPoseGraphProcessor::RunPutProgress(const double& _progress) {
  if (cb_put_progress_) {
    cb_put_progress_(_progress);
  }
}

bool TinyPoseGraphProcessor::IsRunning() { return start_flag_; }

void TinyPoseGraphProcessor::Reset() {
  state_ = TPGS_IDLE;
  loop_closure_detector_ = std::make_shared<LoopClosureDetector>(
      pose_graph_params_->loop_closure_searching_distance,
      pose_graph_params_->loop_closure_distance_interval,
      pose_graph_params_->loop_closure_score_threshold,
      pose_graph_params_->mono_layer_motion);

  pose_graph_manager_ = std::make_shared<PoseGraphManager>();

  lidar_session_ = nullptr;
}

}  // namespace multi_sensor_mapping
