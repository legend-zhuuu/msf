#include "multi_sensor_mapping/core/pose_graph_processor.h"

#include "multi_sensor_mapping/backend/loop_closure_detector.h"
#include "multi_sensor_mapping/backend/pose_graph_manager.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

PoseGraphProcessor::PoseGraphProcessor()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      pose_graph_state_(PGS_UNINITIALIZED),
      update_base_session_flag_(false),
      update_loop_closure_info_flag_(false),
      selected_loop_closure_id_(-1),
      update_slice_param_flag_(0),
      delta_slice_param_(Eigen::Vector3d(0, 0, 0)),
      update_map_yaw_flag_(false),
      delta_map_yaw_(0),
      update_session_flag_(false),
      local_match_cmd_(LocalMatchCmd::LMC_NONE) {}

void PoseGraphProcessor::SetParams(
    const std::shared_ptr<ParamSet>& _param_set) {
  sensor_params_ = _param_set->GetSensorParams();
  extrinsic_params_ = _param_set->GetExtrinsicParams();
  mapper_params_ = _param_set->GetLidarImuMappingParams();
  mapper_params_->Print();
}

void PoseGraphProcessor::Init() {
  AINFO_F("[PoseGraphProcessor] Init Pose-Graph-Processor");

  // 检测参数
  if (!mapper_params_ || !sensor_params_ || !extrinsic_params_) {
    AWARN_F("[PoseGraphProcessor] No input Params");
    RunException(MSMError(CODE_NO_INPUT_PARAMS));
    return;
  }

  map_resolution_ = mapper_params_->map_resolution;

  loop_closure_detector_ = std::make_shared<LoopClosureDetector>(
      mapper_params_->loop_closure_searching_distance,
      mapper_params_->loop_closure_distance_interval,
      mapper_params_->loop_closure_score_threshold,
      mapper_params_->mono_layer_motion);

  pose_graph_manager_ = std::make_shared<PoseGraphManager>();

  init_done_flag_ = true;
}
void PoseGraphProcessor::Start() {
  if (start_flag_) {
    AINFO_F(
        "[PoseGraphProcessor] Pose Graph Process thread have already running");
    return;
  }

  if (!init_done_flag_) {
    AWARN_F("[PoseGraphProcessor] Please init system before start");
    RunException(MSMError(CODE_START_BEFORE_INIT));
    return;
  }
  exit_process_flag_ = false;

  // 主处理线程
  process_thread_ =
      std::thread(std::bind(&PoseGraphProcessor::LoopProcess, this));

  start_flag_ = true;
}

void PoseGraphProcessor::Stop() {
  if (!start_flag_) {
    return;
  }

  exit_process_flag_ = true;
  process_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;
}

bool PoseGraphProcessor::SetBaseMapSession(std::string _session_path) {
  base_map_session_ = std::make_shared<LidarMapSession>();

  base_map_session_->Load(_session_path);

  session_cache_path_ = _session_path;
  update_base_session_flag_ = true;

  return true;
}

bool PoseGraphProcessor::SetBaseMapSession(
    const std::shared_ptr<LidarMapSession>& _session) {
  base_map_session_ = _session;
  session_cache_path_ = base_map_session_->GetCachePath();
  update_base_session_flag_ = true;
  return true;
}

void PoseGraphProcessor::StartAutoLoopClosureDetection() {
  if (pose_graph_state_ == PGS_UNINITIALIZED) return;
  pose_graph_state_ = PoseGraphState::PGS_AUTO_LOOP_DETECTION;
}

void PoseGraphProcessor::StartBackendOptimization() {
  if (pose_graph_state_ == PGS_UNINITIALIZED) return;
  pose_graph_state_ = PoseGraphState::PGS_OPTIMIZATION;
}

void PoseGraphProcessor::AdjustLocalMatch(int _lc_id, double _x, double _y,
                                          double _z, double _roll,
                                          double _pitch, double _yaw) {
  if (pose_graph_state_ != PGS_MANUAL_LOOP_DETECTION) return;

  if (_lc_id != selected_loop_closure_id_) {
    AWARN_F("[PoseGraphProcessor] Local match ID mismatch [ %i, %i ] ", _lc_id,
            selected_loop_closure_id_);
    return;
  }

  adjusted_local_source_pose_ =
      utils::XYZYPR2Transd(_x, _y, _z, _yaw, _pitch, _roll);

  local_match_cmd_ = LocalMatchCmd::LMC_ADJUST;
}

void PoseGraphProcessor::MatchLocalMatch(int _lc_id, double _x, double _y,
                                         double _z, double _roll, double _pitch,
                                         double _yaw) {
  if (pose_graph_state_ != PGS_MANUAL_LOOP_DETECTION) return;

  if (_lc_id != selected_loop_closure_id_) {
    AWARN_F("[PoseGraphProcessor] Local match ID mismatch [ %i, %i ] ", _lc_id,
            selected_loop_closure_id_);
    return;
  }

  adjusted_local_source_pose_ =
      utils::XYZYPR2Transd(_x, _y, _z, _yaw, _pitch, _roll);

  local_match_cmd_ = LocalMatchCmd::LMC_MATCH;
}

void PoseGraphProcessor::UpdateLocalMatch(int _lc_id) {
  if (pose_graph_state_ != PGS_MANUAL_LOOP_DETECTION) return;

  if (_lc_id != selected_loop_closure_id_) {
    AWARN_F("[PoseGraphProcessor] Local match ID mismatch [ %i, %i ] ", _lc_id,
            selected_loop_closure_id_);
    return;
  }

  local_match_cmd_ = LocalMatchCmd::LMC_UPDATE;
}

void PoseGraphProcessor::StartManualLoopClosureDetection(int _lc_id) {
  if (pose_graph_state_ == PGS_UNINITIALIZED) return;
  int loop_closure_num = pose_graph_manager_->pose_graph_database_
                             ->loop_closure_info_id_vec_.size();
  if (_lc_id >= loop_closure_num) {
    AINFO_F("[PoseGraphProcessor] Selected ID is too large [ %i , %i ]", _lc_id,
            loop_closure_num);
    return;
  }
  pose_graph_state_ = PoseGraphState::PGS_MANUAL_LOOP_DETECTION;

  if (selected_loop_closure_id_ != _lc_id) {
    selected_loop_closure_id_ = _lc_id;
    update_loop_closure_info_flag_ = true;
  }
}

void PoseGraphProcessor::SaveLidarMapSession(std::string _save_path) {
  if (pose_graph_state_ == PGS_UNINITIALIZED) return;
  pose_graph_state_ = PoseGraphState::PGS_SAVE;

  if (!_save_path.empty()) {
    session_save_path_ = _save_path;
  }
}

void PoseGraphProcessor::StartPostProcess() {
  pose_graph_state_ = PoseGraphState::PGS_POST_PROCESS;
}

void PoseGraphProcessor::AdjustSliceParam(double _delta_radius,
                                          double _delta_height,
                                          double _delta_thickness) {
  if (pose_graph_state_ != PoseGraphState::PGS_POST_PROCESS) {
    return;
  }
  update_slice_param_flag_ = 1;
  delta_slice_param_ =
      Eigen::Vector3d(_delta_radius, _delta_height, _delta_thickness);
}

void PoseGraphProcessor::AdjustSliceParam2(double _slice_radius,
                                           double _slice_height,
                                           double _slice_thickness) {
  if (pose_graph_state_ != PoseGraphState::PGS_POST_PROCESS) {
    return;
  }

  update_slice_param_flag_ = 2;
  slice_param_ =
      Eigen::Vector3d(_slice_radius, _slice_height, _slice_thickness);
}

void PoseGraphProcessor::AdjustMapYaw(double _delta_yaw) {
  if (pose_graph_state_ != PoseGraphState::PGS_POST_PROCESS) {
    return;
  }

  update_map_yaw_flag_ = true;
  delta_map_yaw_ = _delta_yaw;
}

void PoseGraphProcessor::AdjustMapResolution(double _resolution) {
  if (_resolution > 0) {
    map_resolution_ = _resolution;
    AINFO_F("[PoseGraphProcessor] Adjust Map Resolution < %f >", _resolution);
  }
}

void PoseGraphProcessor::RegExceptionCallback(
    const std::function<void(const MSMError&)>& _cb_excep) {
  cb_exception_ = _cb_excep;
}

void PoseGraphProcessor::RegPoseGraphInfoCallback(
    const std::function<void(const PoseGraph&)>& _cb_pose_graph) {
  cb_pose_graph_ = _cb_pose_graph;
}

void PoseGraphProcessor::RegGlobalCloudCallback(
    const std::function<void(const CloudTypePtr&)>& _cb_global_cloud) {
  cb_global_cloud_ = _cb_global_cloud;
}

void PoseGraphProcessor::RegLocalMatchInfoCallback(
    const std::function<void(const LocalMatchInformation&)>& _cb_local_match) {
  cb_local_match_info_ = _cb_local_match;
}

void PoseGraphProcessor::RegProcessState(
    const std::function<void(const ProcessState&)>& _cb_process_state) {
  cb_process_state_ = _cb_process_state;
}

void PoseGraphProcessor::UpdateMapSession() {
  if (pose_graph_state_ != PoseGraphState::PGS_POST_PROCESS) {
    return;
  }
  update_session_flag_ = true;
}

bool PoseGraphProcessor::IsRunning() { return start_flag_; }

void PoseGraphProcessor::RunException(const MSMError& _error) {
  if (cb_exception_) {
    cb_exception_(_error);
  }
}

void PoseGraphProcessor::LoopProcess() {
  double upload_pose_graph_time = 0;
  double loop_time;

  LocalMatchInformation active_local_match_info;
  Eigen::Matrix4d target_pose_in_local;

  while (!exit_process_flag_) {
    loop_time = GetSystemTimeSecond();

    if (pose_graph_state_ == PGS_UNINITIALIZED) {
      if (update_base_session_flag_) {
        update_base_session_flag_ = false;
        if (base_map_session_) {
          pose_graph_manager_->ImportGraphFromMapSession(base_map_session_);

          // 上传位子图和全局点云
          if (cb_pose_graph_) {
            PoseGraph graph = base_map_session_->ExportPoseGraph();
            cb_pose_graph_(graph);
            AINFO_F("[PoseGraphProcessor] Upload pose graph ");
          }

          // if (cb_global_cloud_) {
          //   CloudTypePtr global_cloud = base_map_session_->GetGlobalMap();
          //   cb_global_cloud_(global_cloud);
          //   AINFO_F("[PoseGraphProcessor] Upload global cloud");
          // }
          upload_pose_graph_time = loop_time;

          // 状态切换
          if (mapper_params_->auto_loop_closure) {
            pose_graph_state_ = PGS_AUTO_BACKEND;
          } else {
            pose_graph_state_ = PGS_IDLE;
          }
          AINFO_F("[PoseGraphProcessor] Load Map Session Success!");
        }
        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }
      }
    } else if (pose_graph_state_ == PGS_IDLE) {
      if (loop_time - upload_pose_graph_time > 5) {
        if (base_map_session_ && cb_pose_graph_) {
          PoseGraph graph = base_map_session_->ExportPoseGraph();
          cb_pose_graph_(graph);
        }

        // 若地图很大，该步骤比较耗时，故注释掉
        // if (base_map_session_ && cb_global_cloud_) {
        //   CloudTypePtr global_cloud = base_map_session_->GetGlobalMap();
        //   cb_global_cloud_(global_cloud);
        // }
        upload_pose_graph_time = loop_time;
      }
    } else if (pose_graph_state_ == PGS_AUTO_LOOP_DETECTION) {
      AINFO_F("[PoseGraphProcessor] Start Auto-Loop-Closure-Detection !");

      AutoLoopDetection(base_map_session_);
      AINFO_F("[PoseGraphProcessor] Complete Auto-Loop-Closure-Detection !");

      pose_graph_state_ = PGS_IDLE;
      if (cb_process_state_) {
        cb_process_state_(ProcessState::PROCESS_FINSIH);
      }
    } else if (pose_graph_state_ == PGS_OPTIMIZATION) {
      AINFO_F("[PoseGraphProcessor] Start Back-End-Optimization !");

      BackendOptimization();

      // 优化完成后可视化地图
      // if (base_map_session_ && cb_global_cloud_) {
      //   CloudTypePtr global_cloud = base_map_session_->GetGlobalMap();
      //   cb_global_cloud_(global_cloud);
      // }

      AINFO_F("[PoseGraphProcessor] Complete Back-End-Optimization !");
      pose_graph_state_ = PGS_IDLE;
      if (cb_process_state_) {
        cb_process_state_(ProcessState::PROCESS_FINSIH);
      }
    } else if (pose_graph_state_ == PGS_MANUAL_LOOP_DETECTION) {
      // 更新闭环信息
      if (update_loop_closure_info_flag_) {
        base_map_session_->GetLocalMatchInformation(selected_loop_closure_id_,
                                                    active_local_match_info);

        AINFO_F("[PoseGraphProcessor] Select loop Closure [ %i ], score [ %f ]",
                selected_loop_closure_id_, active_local_match_info.score);

        target_pose_in_local = active_local_match_info.target_pose_in_map;
        target_pose_in_local.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0);

        if (cb_local_match_info_) {
          cb_local_match_info_(active_local_match_info);
        }

        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }

        update_loop_closure_info_flag_ = false;
      }

      if (local_match_cmd_ == LocalMatchCmd::LMC_ADJUST) {
        // 调整local match
        Eigen::Matrix4d delta_pose =
            target_pose_in_local.inverse() * adjusted_local_source_pose_;
        active_local_match_info.relative_pose = delta_pose;
        if (cb_local_match_info_) {
          cb_local_match_info_(active_local_match_info);
        }

        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }

        local_match_cmd_ = LocalMatchCmd::LMC_NONE;
      } else if (local_match_cmd_ == LocalMatchCmd::LMC_MATCH) {
        // 匹配local match
        Eigen::Matrix4d delta_pose =
            target_pose_in_local.inverse() * adjusted_local_source_pose_;

        float reg_score;
        loop_closure_detector_->CloudRevalidation(
            active_local_match_info.target_cloud.makeShared(),
            active_local_match_info.source_cloud.makeShared(), delta_pose,
            active_local_match_info.relative_pose, reg_score);
        AINFO_F("[PoseGraphProcessor] Local Match Score [ %f ]", reg_score);
        active_local_match_info.score = reg_score;
        if (cb_local_match_info_) {
          cb_local_match_info_(active_local_match_info);
        }

        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }

        local_match_cmd_ = LocalMatchCmd::LMC_NONE;
      } else if (local_match_cmd_ == LocalMatchCmd::LMC_UPDATE) {
        // 更新 loop closure
        pose_graph_manager_->UpdateLoopClosureEdge(
            active_local_match_info.loop_closure_id,
            active_local_match_info.relative_pose,
            active_local_match_info.score);

        // 上传更新后的pose graph
        base_map_session_->GetLocalMatchInformation(selected_loop_closure_id_,
                                                    active_local_match_info);

        if (cb_local_match_info_) {
          cb_local_match_info_(active_local_match_info);
        }

        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }

        AINFO_F("[PoseGraphProcessor] Complete update pose graph < %i >",
                selected_loop_closure_id_);

        local_match_cmd_ = LocalMatchCmd::LMC_NONE;
      }
    } else if (pose_graph_state_ == PGS_SAVE) {
      AINFO_F("[PoseGraphProcessor] Start Save Lidar Map Session !");

      if (session_save_path_.empty()) {
        auto now = std::time(nullptr);
        auto now_time = std::localtime(&now);
        char buffer[9];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d", now_time);
        std::string time_str(buffer);

        session_save_path_ = session_cache_path_ + "(" + time_str + ")";
      }

      base_map_session_->Save(session_save_path_, map_resolution_);
      pose_graph_state_ = PGS_IDLE;
      if (cb_process_state_) {
        cb_process_state_(ProcessState::PROCESS_FINSIH);
      }
      AINFO_F("[PoseGraphProcessor] Complete Save Lidar Map Session [ %s ] !",
              session_save_path_.c_str());
    } else if (pose_graph_state_ == PGS_POST_PROCESS) {
      if (update_slice_param_flag_ == 1) {
        // 更新slice参数
        AINFO_F(
            "[PoseGraphProcessor] Adjust Slice Cloud Param [ %f, %f, %f ] !",
            delta_slice_param_(0), delta_slice_param_(1),
            delta_slice_param_(2));
        update_slice_param_flag_ = 0;
        CloudTypePtr slice_cloud = base_map_session_->ExtractTopViewCloud(
            delta_slice_param_(0), delta_slice_param_(1),
            delta_slice_param_(2));

        Eigen::Matrix4d update_pose = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond map_q(Eigen::AngleAxisd(
            delta_map_yaw_ * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
        update_pose.block<3, 3>(0, 0) = map_q.toRotationMatrix();
        CloudTypePtr slice_cloud_adjusted(new CloudType);
        pcl::transformPointCloud(*slice_cloud, *slice_cloud_adjusted,
                                 update_pose);

        if (cb_global_cloud_) {
          cb_global_cloud_(slice_cloud_adjusted);
        }
        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }
      } else if (update_slice_param_flag_ == 2) {
        // 更新slice参数
        AINFO_F(
            "[PoseGraphProcessor] Adjust Slice Cloud Param [ %f, %f, %f ] !",
            slice_param_(0), slice_param_(1), slice_param_(2));
        update_slice_param_flag_ = -1;
        CloudTypePtr slice_cloud = base_map_session_->ExtractTopViewCloud2(
            slice_param_(0), slice_param_(1), slice_param_(2));

        Eigen::Matrix4d update_pose = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond map_q(Eigen::AngleAxisd(
            delta_map_yaw_ * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
        update_pose.block<3, 3>(0, 0) = map_q.toRotationMatrix();
        CloudTypePtr slice_cloud_adjusted(new CloudType);
        pcl::transformPointCloud(*slice_cloud, *slice_cloud_adjusted,
                                 update_pose);

        if (cb_global_cloud_) {
          cb_global_cloud_(slice_cloud_adjusted);
        }
        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }
      }

      if (update_map_yaw_flag_) {
        AINFO_F("[PoseGraphProcessor] Adjust Map Yaw Param [ %f ] !",
                delta_map_yaw_);
        update_map_yaw_flag_ = false;

        Eigen::Matrix4d update_pose = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond map_q(Eigen::AngleAxisd(
            delta_map_yaw_ * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
        update_pose.block<3, 3>(0, 0) = map_q.toRotationMatrix();

        CloudTypePtr slice_cloud;
        if (update_slice_param_flag_ == 0) {
          slice_cloud = base_map_session_->ExtractTopViewCloud(
              delta_slice_param_(0), delta_slice_param_(1),
              delta_slice_param_(2));
        } else if (update_slice_param_flag_ == -1) {
          slice_cloud = base_map_session_->ExtractTopViewCloud2(
              slice_param_(0), slice_param_(1), slice_param_(2));
        }

        CloudTypePtr slice_cloud_adjusted(new CloudType);
        pcl::transformPointCloud(*slice_cloud, *slice_cloud_adjusted,
                                 update_pose);
        if (cb_global_cloud_) {
          cb_global_cloud_(slice_cloud_adjusted);
        }
        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }
      }

      if (update_session_flag_) {
        AINFO_F("[PoseGraphProcessor] Update Session Param !");
        update_session_flag_ = false;
        if (update_slice_param_flag_ == 0) {
          base_map_session_->UpdateSliceParam(delta_slice_param_);
        } else if (update_slice_param_flag_ == -1) {
          base_map_session_->UpdateSliceParam(slice_param_, false);
        }
        base_map_session_->UpdateMapYaw(delta_map_yaw_);

        pose_graph_state_ = PGS_IDLE;

        if (cb_process_state_) {
          cb_process_state_(ProcessState::PROCESS_FINSIH);
        }
      }
    } else if (pose_graph_state_ == PGS_AUTO_BACKEND) {
      AINFO_F("[PoseGraphProcessor] Start Auto-Backend !");
      // 闭环检测
      AutoLoopDetection(base_map_session_);
      // 闭环优化
      BackendOptimization();
      // 保存地图
      base_map_session_->Save(session_cache_path_,
                              mapper_params_->map_resolution);
      AINFO_F("[PoseGraphProcessor] Complete Auto-Backend !");

      pose_graph_state_ = PGS_IDLE;
    }
    usleep(10000);
  }
}

void PoseGraphProcessor::AutoLoopDetection(
    const std::shared_ptr<LidarMapSession>& _session) {
  loop_closure_detector_->SetLidarMapSession(_session);

  int num_map_unit = _session->GetMapUnitSize();

  pose_graph_manager_->ClearLoopClosure();

  for (int i = 0; i < num_map_unit; i++) {
    LoopClosureResult loop_closure_result =
        loop_closure_detector_->DetectLoopClosure(i);
    if (loop_closure_result.success_flag == true) {
      AINFO_F(
          "[PoseGraphProcessor] Loop closure detect success : %i ---> %i  , "
          "score < %f >",
          loop_closure_result.current_scan_id_,
          loop_closure_result.history_scan_id_, loop_closure_result.score_);
      // 添加闭环边
      pose_graph_manager_->AddLoopClosureEdge(
          loop_closure_result.history_scan_id_,
          loop_closure_result.current_scan_id_,
          loop_closure_result.trans_history_to_current,
          loop_closure_result.score_);
    } else {
      if (loop_closure_result.history_scan_id_ < 0 ||
          loop_closure_result.current_scan_id_ < 0) {
        continue;
      }
      pose_graph_manager_->AddLoopClosureEdge(
          loop_closure_result.history_scan_id_,
          loop_closure_result.current_scan_id_,
          loop_closure_result.trans_history_to_current,
          loop_closure_result.score_, false);
    }
  }

  // _session->SaveLoopClosureCloud();
}

void PoseGraphProcessor::BackendOptimization() {
  pose_graph_manager_->BuildProblem();
  pose_graph_manager_->SolveProblem(true, 100);
  pose_graph_manager_->UpdateLidarMapSession(base_map_session_);
}

}  // namespace multi_sensor_mapping