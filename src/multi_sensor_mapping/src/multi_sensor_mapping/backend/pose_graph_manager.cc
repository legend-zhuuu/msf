#include "multi_sensor_mapping/backend/pose_graph_manager.h"

#include "multi_sensor_mapping/backend/pose_graph_information.h"
#include "multi_sensor_mapping/factor/auto_diff/pose_graph_factor.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

PoseGraphManager::PoseGraphManager()
    : scan_match_t_var_(0.01),
      scan_match_q_var_(0.01),
      loop_closure_t_var_(0.01),
      loop_closure_q_var_(0.01),
      gps_edge_flag_(true),
      latest_loop_closure_scan_id_(0) {
  solve_options_.linear_solver_type = ceres::DENSE_SCHUR;
  solve_options_.max_num_iterations = 100.0;
  solve_options_.num_threads = 10;

  pose_graph_database_ = std::make_shared<PoseGraphDatabase>();

  local_parameterization_ = new ceres::QuaternionParameterization();

  p_GinL_[0] = 0;
  p_GinL_[1] = 0;
  p_GinL_[2] = 0;
}

void PoseGraphManager::SetPoseGraphDatabase(
    std::shared_ptr<PoseGraphDatabase> _pgd_ptr) {
  pose_graph_database_ = _pgd_ptr;
}

void PoseGraphManager::SetExtGNSS2Lidar(
    const Eigen::Vector3d &_ext_translation) {
  p_GinL_[0] = _ext_translation[0];
  p_GinL_[1] = _ext_translation[1];
  p_GinL_[2] = _ext_translation[2];

  gps_edge_flag_ = true;
}

void PoseGraphManager::SetExtGNSS2Lidar(const Eigen::Matrix4d &_ext_matrix) {
  p_GinL_[0] = _ext_matrix(0, 3);
  p_GinL_[1] = _ext_matrix(1, 3);
  p_GinL_[2] = _ext_matrix(2, 3);

  gps_edge_flag_ = true;
}

void PoseGraphManager::AddPoseNode(const int _id,
                                   const Eigen::Vector3d &_pose_t,
                                   const Eigen::Quaterniond &_pose_q) {
  if (pose_params_map_.count(_id) > 0) {
    AINFO_F("[PoseGraphManager] Please check pose id first");
  }

  PoseParams cur_pose;
  cur_pose.t_param[0] = _pose_t[0];
  cur_pose.t_param[1] = _pose_t[1];
  cur_pose.t_param[2] = _pose_t[2];

  cur_pose.q_param[0] = _pose_q.w();
  cur_pose.q_param[1] = _pose_q.x();
  cur_pose.q_param[2] = _pose_q.y();
  cur_pose.q_param[3] = _pose_q.z();

  pose_params_map_[_id] = cur_pose;

  pose_id_vec_.push_back(_id);
}

void PoseGraphManager::AddPoseNode(const int _id,
                                   const Eigen::Matrix4d &_pose) {
  if (pose_params_map_.count(_id) > 0) {
    AINFO_F("[PoseGraphManager] Please check pose id first");
  }

  PoseParams cur_pose;
  cur_pose.t_param[0] = _pose(0, 3);
  cur_pose.t_param[1] = _pose(1, 3);
  cur_pose.t_param[2] = _pose(2, 3);

  Eigen::Quaterniond pose_q(_pose.block<3, 3>(0, 0));
  cur_pose.q_param[0] = pose_q.w();
  cur_pose.q_param[1] = pose_q.x();
  cur_pose.q_param[2] = pose_q.y();
  cur_pose.q_param[3] = pose_q.z();

  pose_params_map_[_id] = cur_pose;

  pose_id_vec_.push_back(_id);
}

void PoseGraphManager::SetPoseNodes(
    const std::vector<Eigen::Matrix4d> &_pose_vec) {
  pose_params_map_.clear();
  pose_id_vec_.clear();

  for (int i = 0; i < (int)_pose_vec.size(); i++) {
    AddPoseNode(i, _pose_vec[i]);
  }
}

void PoseGraphManager::AddLoopClosureEdge(
    const int _front_id, const int _back_id, const Eigen::Vector3d &_delta_t,
    const Eigen::Quaterniond &_delta_q, const double _score, bool _valid_flag) {
  if (pose_params_map_.count(_front_id) == 0 ||
      pose_params_map_.count(_back_id) == 0) {
    AINFO_F("[PoseGraphManager] Please Check pose id first  %i, <---> %i ",
            _front_id, _back_id);

    return;
  }

  latest_loop_closure_scan_id_ = _back_id;

  LoopClosureConstraint loop_closure_constraint;
  loop_closure_constraint.front_id = _front_id;
  loop_closure_constraint.back_id = _back_id;
  loop_closure_constraint.delta_position = _delta_t;
  loop_closure_constraint.delta_rotation = _delta_q;
  loop_closure_constraint.valid_flag = _valid_flag;
  loop_closure_constraint.reg_score = _score;

  // 存入数据库中
  pose_graph_database_->AddLoopClosureConstraint(loop_closure_constraint);
}

void PoseGraphManager::AddLoopClosureEdge(const int _front_id,
                                          const int _back_id,
                                          const Eigen::Matrix4d &_delta_pose,
                                          const double _score,
                                          bool _valid_flag) {
  if (pose_params_map_.count(_front_id) == 0 ||
      pose_params_map_.count(_back_id) == 0) {
    AINFO_F("[PoseGraphManager] Please Check pose id first  %i, <---> %i ",
            _front_id, _back_id);
    return;
  }
  latest_loop_closure_scan_id_ = _back_id;

  Eigen::Quaterniond delta_q(_delta_pose.block<3, 3>(0, 0));

  LoopClosureConstraint loop_closure_constraint;
  loop_closure_constraint.front_id = _front_id;
  loop_closure_constraint.back_id = _back_id;
  loop_closure_constraint.delta_position =
      Eigen::Vector3d(_delta_pose(0, 3), _delta_pose(1, 3), _delta_pose(2, 3));
  loop_closure_constraint.delta_rotation = delta_q.template cast<double>();
  loop_closure_constraint.valid_flag = _valid_flag;
  loop_closure_constraint.reg_score = _score;

  // 存入数据库中
  pose_graph_database_->AddLoopClosureConstraint(loop_closure_constraint);
}

void PoseGraphManager::AddGpsEdge(const int _id,
                                  const Eigen::Vector3d &_gps_pose,
                                  const Eigen::Vector3d &_gps_noise,
                                  bool _use_elevation_flag) {
  if (pose_params_map_.count(_id) == 0) {
    AINFO_F("[PoseGraphManager] Please check pose id first");
    return;
  }

  GNSSConstraint gnss_constrait;
  gnss_constrait.id = _id;
  gnss_constrait.gnss_pose = _gps_pose;
  gnss_constrait.gnss_noise = _gps_noise;
  gnss_constrait.use_gps_elevation = _use_elevation_flag;
  gnss_constrait.valid_flag = true;

  pose_graph_database_->AddGnssConstraint(gnss_constrait);
}

void PoseGraphManager::UpdateLoopClosureEdge(
    const int _lc_id, const Eigen::Matrix4d &_relative_pose, int _score) {
  auto iter = pose_graph_database_->loop_closure_constraint_info_.find(_lc_id);

  if (iter == pose_graph_database_->loop_closure_constraint_info_.end()) {
    AWARN_F(
        "[PoseGraphManager] Unable to find the ID < %i > in the pose graph "
        "database",
        _lc_id);
  } else {
    Eigen::Quaterniond delta_q(_relative_pose.block<3, 3>(0, 0));
    iter->second.delta_position = Eigen::Vector3d(
        _relative_pose(0, 3), _relative_pose(1, 3), _relative_pose(2, 3));
    iter->second.delta_rotation = delta_q;
    iter->second.reg_score = _score;
    iter->second.valid_flag = true;
  }
}

void PoseGraphManager::BuildProblem() {
  //  problem_ = std::shared_ptr<ceres::Problem>(new ceres::Problem);
  problem_.reset(new ceres::Problem);
  local_parameterization_ = new ceres::QuaternionParameterization();

  // 添加位姿节点
  for (const auto iterm : pose_id_vec_) {
    problem_->AddParameterBlock(pose_params_map_[iterm].t_param, 3);
    problem_->AddParameterBlock(pose_params_map_[iterm].q_param, 4,
                                local_parameterization_);
  }

  // 添加帧间约束
  int latest_id;
  Eigen::Vector3d latest_position;
  Eigen::Quaterniond latest_rotation;
  for (size_t i = 0; i < pose_id_vec_.size(); i++) {
    if (i == 0) {
      latest_id = pose_id_vec_[i];
      latest_position = Eigen::Vector3d(pose_params_map_[latest_id].t_param[0],
                                        pose_params_map_[latest_id].t_param[1],
                                        pose_params_map_[latest_id].t_param[2]);
      latest_rotation =
          Eigen::Quaterniond(pose_params_map_[latest_id].q_param[0],
                             pose_params_map_[latest_id].q_param[1],
                             pose_params_map_[latest_id].q_param[2],
                             pose_params_map_[latest_id].q_param[3]);
    } else {
      int cur_id = pose_id_vec_[i];
      Eigen::Vector3d cur_position =
          Eigen::Vector3d(pose_params_map_[cur_id].t_param[0],
                          pose_params_map_[cur_id].t_param[1],
                          pose_params_map_[cur_id].t_param[2]);
      Eigen::Quaterniond cur_rotation =
          Eigen::Quaterniond(pose_params_map_[cur_id].q_param[0],
                             pose_params_map_[cur_id].q_param[1],
                             pose_params_map_[cur_id].q_param[2],
                             pose_params_map_[cur_id].q_param[3]);
      Eigen::Vector3d relative_t =
          latest_rotation.inverse() * (cur_position - latest_position);
      Eigen::Quaterniond relative_q = latest_rotation.inverse() * cur_rotation;

      ceres::CostFunction *functor = RelativeRTFactor::Create(
          relative_t(0), relative_t(1), relative_t(2), relative_q.w(),
          relative_q.x(), relative_q.y(), relative_q.z(), scan_match_t_var_,
          scan_match_q_var_);
      problem_->AddResidualBlock(
          functor, NULL, pose_params_map_[latest_id].q_param,
          pose_params_map_[latest_id].t_param, pose_params_map_[cur_id].q_param,
          pose_params_map_[cur_id].t_param);

      latest_id = cur_id;
      latest_position = cur_position;
      latest_rotation = cur_rotation;
    }
  }

  // 添加GNSS约束
  if (pose_graph_database_->GnssConstraintEmpty()) {
    problem_->SetParameterBlockConstant(
        pose_params_map_[pose_id_vec_[0]].q_param);
    problem_->SetParameterBlockConstant(
        pose_params_map_[pose_id_vec_[0]].t_param);
  } else {
    if (!gps_edge_flag_) {
      AWARN_F("[PoseGraphManager] Please set p_GinL first");
    }

    problem_->AddParameterBlock(p_GinL_, 3);
    problem_->SetParameterBlockConstant(p_GinL_);

    for (int id : pose_graph_database_->gnss_constraint_id_vec_) {
      GNSSConstraint gnss_constraint =
          pose_graph_database_->gnss_constraint_info_[id];
      if (!gnss_constraint.valid_flag) continue;

      // gnss_constraint.gnss_noise[0] = 0.1;
      // gnss_constraint.gnss_noise[1] = 0.1;
      // gnss_constraint.gnss_noise[2] = 0.2;

      // 添加GPS的残差项
      if (gnss_constraint.use_gps_elevation) {
        ceres::CostFunction *functor = AnchorPointFactor::Create(
            gnss_constraint.gnss_pose[0], gnss_constraint.gnss_pose[1],
            gnss_constraint.gnss_pose[2], gnss_constraint.gnss_noise[0],
            gnss_constraint.gnss_noise[1], gnss_constraint.gnss_noise[2]);
        problem_->AddResidualBlock(
            functor, NULL, pose_params_map_[gnss_constraint.id].q_param,
            pose_params_map_[gnss_constraint.id].t_param, p_GinL_);
      } else {
        ceres::CostFunction *functor = AnchorPointFactor2::Create(
            gnss_constraint.gnss_pose[0], gnss_constraint.gnss_pose[1],
            gnss_constraint.gnss_noise[0], gnss_constraint.gnss_noise[1]);
        problem_->AddResidualBlock(
            functor, NULL, pose_params_map_[gnss_constraint.id].q_param,
            pose_params_map_[gnss_constraint.id].t_param, p_GinL_);
      }
    }
  }

  // 添加闭环约束
  if (!pose_graph_database_->LoopClosureConstraintEmpty()) {
    for (int id : pose_graph_database_->loop_closure_info_id_vec_) {
      LoopClosureConstraint loop_closure_constraint =
          pose_graph_database_->loop_closure_constraint_info_[id];
      if (!loop_closure_constraint.valid_flag) continue;

      // 添加闭环残差项
      ceres::CostFunction *functor =
          RelativeRTFactor::Create(loop_closure_constraint.delta_position(0),
                                   loop_closure_constraint.delta_position(1),
                                   loop_closure_constraint.delta_position(2),
                                   loop_closure_constraint.delta_rotation.w(),
                                   loop_closure_constraint.delta_rotation.x(),
                                   loop_closure_constraint.delta_rotation.y(),
                                   loop_closure_constraint.delta_rotation.z(),
                                   loop_closure_t_var_, loop_closure_q_var_);
      problem_->AddResidualBlock(
          functor, NULL,
          pose_params_map_[loop_closure_constraint.front_id].q_param,
          pose_params_map_[loop_closure_constraint.front_id].t_param,
          pose_params_map_[loop_closure_constraint.back_id].q_param,
          pose_params_map_[loop_closure_constraint.back_id].t_param);
    }
  }

  AINFO_F("[PoseGraphManager] Create ceres problem done !");
}

void PoseGraphManager::SolveProblem(bool _std_out, int _max_iter) {
  solve_options_.max_num_iterations = _max_iter;
  solve_options_.minimizer_progress_to_stdout = _std_out;
  solve_options_.num_threads = 6;

  ceres::Solver::Summary summary;
  ceres::Solve(solve_options_, problem_.get(), &summary);

  AINFO_F("[PoseGraphManager] Summary [ %s ] ", summary.BriefReport().c_str());
}

void PoseGraphManager::ImportGraphFromMapSession(
    const std::shared_ptr<LidarMapSession> &_map_session) {
  // 导如节点信息
  int num_map_units = _map_session->GetMapUnitSize();
  for (int i = 0; i < num_map_units; i++) {
    Eigen::Matrix4d unit_pose = _map_session->GetMapUnitPose(i);
    AddPoseNode(i, unit_pose);
  }

  if (_map_session->pose_graph_database_ptr_) {
    pose_graph_database_ = _map_session->pose_graph_database_ptr_;
  }
}

void PoseGraphManager::UpdateLidarMapSession(
    std::shared_ptr<LidarMapSession> &_map_session) {
  std::map<int, PoseParams>::iterator iter;
  for (iter = pose_params_map_.begin(); iter != pose_params_map_.end();
       iter++) {
    Eigen::Vector3d updated_translation(iter->second.t_param[0],
                                        iter->second.t_param[1],
                                        iter->second.t_param[2]);
    Eigen::Quaterniond updated_rotation(
        iter->second.q_param[0], iter->second.q_param[1],
        iter->second.q_param[2], iter->second.q_param[3]);

    _map_session->UpdateMapUnitPose(iter->first, updated_translation,
                                    updated_rotation);
  }

  // 共享位姿图数据库
  _map_session->pose_graph_database_ptr_ = pose_graph_database_;
}

void PoseGraphManager::ClearLoopClosure() {
  pose_graph_database_->ClearLoopClosureConstraint();
}

void PoseGraphManager::IncrementalPoseGraphOptimization() {
  problem_.reset(new ceres::Problem);
  local_parameterization_ = new ceres::QuaternionParameterization();

  Eigen::Vector3d temp_latest_loop_closure_position = Eigen::Vector3d(
      pose_params_map_[latest_loop_closure_scan_id_].t_param[0],
      pose_params_map_[latest_loop_closure_scan_id_].t_param[1],
      pose_params_map_[latest_loop_closure_scan_id_].t_param[2]);

  Eigen::Quaterniond temp_latest_loop_closure_rotation = Eigen::Quaterniond(
      pose_params_map_[latest_loop_closure_scan_id_].q_param[0],
      pose_params_map_[latest_loop_closure_scan_id_].q_param[1],
      pose_params_map_[latest_loop_closure_scan_id_].q_param[2],
      pose_params_map_[latest_loop_closure_scan_id_].q_param[3]);

  Eigen::Matrix4d temp_latest_loop_closure_mat = Eigen::Matrix4d::Identity();
  temp_latest_loop_closure_mat.block<3, 3>(0, 0) =
      temp_latest_loop_closure_rotation.toRotationMatrix();
  temp_latest_loop_closure_mat.block<3, 1>(0, 3) =
      temp_latest_loop_closure_position;

  // 添加位姿节点
  for (const auto iterm : pose_id_vec_) {
    // 只优化最新的闭环节点前的pose grah
    if (iterm <= latest_loop_closure_scan_id_) {
      problem_->AddParameterBlock(pose_params_map_[iterm].t_param, 3);
      problem_->AddParameterBlock(pose_params_map_[iterm].q_param, 4,
                                  local_parameterization_);
    }
  }

  // 添加帧间约束
  int last_id;
  Eigen::Vector3d last_position;
  Eigen::Quaterniond last_rotation;
  for (size_t i = 0; i < pose_id_vec_.size(); i++) {
    if (i == 0) {
      last_id = pose_id_vec_[i];
      last_position = Eigen::Vector3d(pose_params_map_[last_id].t_param[0],
                                      pose_params_map_[last_id].t_param[1],
                                      pose_params_map_[last_id].t_param[2]);
      last_rotation = Eigen::Quaterniond(pose_params_map_[last_id].q_param[0],
                                         pose_params_map_[last_id].q_param[1],
                                         pose_params_map_[last_id].q_param[2],
                                         pose_params_map_[last_id].q_param[3]);
    } else if (i <= latest_loop_closure_scan_id_) {
      int cur_id = pose_id_vec_[i];
      Eigen::Vector3d cur_position =
          Eigen::Vector3d(pose_params_map_[cur_id].t_param[0],
                          pose_params_map_[cur_id].t_param[1],
                          pose_params_map_[cur_id].t_param[2]);
      Eigen::Quaterniond cur_rotation =
          Eigen::Quaterniond(pose_params_map_[cur_id].q_param[0],
                             pose_params_map_[cur_id].q_param[1],
                             pose_params_map_[cur_id].q_param[2],
                             pose_params_map_[cur_id].q_param[3]);
      Eigen::Vector3d relative_t =
          last_rotation.inverse() * (cur_position - last_position);
      Eigen::Quaterniond relative_q = last_rotation.inverse() * cur_rotation;

      ceres::CostFunction *functor = RelativeRTFactor::Create(
          relative_t(0), relative_t(1), relative_t(2), relative_q.w(),
          relative_q.x(), relative_q.y(), relative_q.z(), scan_match_t_var_,
          scan_match_q_var_);
      problem_->AddResidualBlock(
          functor, NULL, pose_params_map_[last_id].q_param,
          pose_params_map_[last_id].t_param, pose_params_map_[cur_id].q_param,
          pose_params_map_[cur_id].t_param);

      last_id = cur_id;
      last_position = cur_position;
      last_rotation = cur_rotation;
    }
  }

  // 添加闭环约束
  if (!pose_graph_database_->LoopClosureConstraintEmpty()) {
    for (int id : pose_graph_database_->loop_closure_info_id_vec_) {
      LoopClosureConstraint loop_closure_constraint =
          pose_graph_database_->loop_closure_constraint_info_[id];
      if (!loop_closure_constraint.valid_flag) continue;

      // 添加闭环残差项
      ceres::CostFunction *functor =
          RelativeRTFactor::Create(loop_closure_constraint.delta_position(0),
                                   loop_closure_constraint.delta_position(1),
                                   loop_closure_constraint.delta_position(2),
                                   loop_closure_constraint.delta_rotation.w(),
                                   loop_closure_constraint.delta_rotation.x(),
                                   loop_closure_constraint.delta_rotation.y(),
                                   loop_closure_constraint.delta_rotation.z(),
                                   loop_closure_t_var_, loop_closure_q_var_);
      problem_->AddResidualBlock(
          functor, NULL,
          pose_params_map_[loop_closure_constraint.front_id].q_param,
          pose_params_map_[loop_closure_constraint.front_id].t_param,
          pose_params_map_[loop_closure_constraint.back_id].q_param,
          pose_params_map_[loop_closure_constraint.back_id].t_param);
    }
  }

  // AINFO_F("[PoseGraphManager] Create ceres problem done !");

  // 优化
  solve_options_.max_num_iterations = 100;
  solve_options_.minimizer_progress_to_stdout = false;
  solve_options_.num_threads = 2;

  ceres::Solver::Summary summary;
  ceres::Solve(solve_options_, problem_.get(), &summary);

  // 更新未优化的姿态
  Eigen::Vector3d current_latest_loop_closure_position = Eigen::Vector3d(
      pose_params_map_[latest_loop_closure_scan_id_].t_param[0],
      pose_params_map_[latest_loop_closure_scan_id_].t_param[1],
      pose_params_map_[latest_loop_closure_scan_id_].t_param[2]);

  Eigen::Quaterniond current_latest_loop_closure_rotation = Eigen::Quaterniond(
      pose_params_map_[latest_loop_closure_scan_id_].q_param[0],
      pose_params_map_[latest_loop_closure_scan_id_].q_param[1],
      pose_params_map_[latest_loop_closure_scan_id_].q_param[2],
      pose_params_map_[latest_loop_closure_scan_id_].q_param[3]);

  Eigen::Matrix4d current_latest_loop_closure_mat = Eigen::Matrix4d::Identity();
  current_latest_loop_closure_mat.block<3, 3>(0, 0) =
      current_latest_loop_closure_rotation.toRotationMatrix();
  current_latest_loop_closure_mat.block<3, 1>(0, 3) =
      current_latest_loop_closure_position;
  Eigen::Matrix4d delta_mat =
      current_latest_loop_closure_mat * temp_latest_loop_closure_mat.inverse();

  for (size_t i = 0; i < pose_id_vec_.size(); i++) {
    int cur_id = pose_id_vec_[i];
    if (cur_id > latest_loop_closure_scan_id_) {
      Eigen::Vector3d cur_position =
          Eigen::Vector3d(pose_params_map_[cur_id].t_param[0],
                          pose_params_map_[cur_id].t_param[1],
                          pose_params_map_[cur_id].t_param[2]);
      Eigen::Quaterniond cur_rotation =
          Eigen::Quaterniond(pose_params_map_[cur_id].q_param[0],
                             pose_params_map_[cur_id].q_param[1],
                             pose_params_map_[cur_id].q_param[2],
                             pose_params_map_[cur_id].q_param[3]);
      Eigen::Matrix4d cut_mat = Eigen::Matrix4d::Identity();
      cut_mat.block<3, 3>(0, 0) = cur_rotation.toRotationMatrix();
      cut_mat.block<3, 1>(0, 3) = cur_position;

      Eigen::Matrix4d updated_mat = delta_mat * cut_mat;

      pose_params_map_[cur_id].t_param[0] = updated_mat(0, 3);
      pose_params_map_[cur_id].t_param[1] = updated_mat(1, 3);
      pose_params_map_[cur_id].t_param[2] = updated_mat(2, 3);
      Eigen::Quaterniond pose_q(updated_mat.block<3, 3>(0, 0));
      pose_q.normalize();
      pose_params_map_[cur_id].q_param[0] = pose_q.w();
      pose_params_map_[cur_id].q_param[1] = pose_q.x();
      pose_params_map_[cur_id].q_param[2] = pose_q.y();
      pose_params_map_[cur_id].q_param[3] = pose_q.z();
    }
  }
}

}  // namespace multi_sensor_mapping
