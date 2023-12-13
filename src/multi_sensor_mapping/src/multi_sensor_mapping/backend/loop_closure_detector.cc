#include "multi_sensor_mapping/backend/loop_closure_detector.h"

#include <algorithm>

#include "multi_sensor_mapping/frontend/lidar/gicp_scan_matcher.h"
#include "multi_sensor_mapping/frontend/lidar/ndt_scan_matcher.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

const int history_key_frame_search_num = 10;

LoopClosureDetector::LoopClosureDetector()
    : loop_closure_searching_distance_(10.0),
      loop_closure_distance_interval_(15.0),
      loop_closure_score_threshold_(0.5),
      mono_layer_motion_(true),
      kdtree_history_pose_(new pcl::KdTreeFLANN<PointType>()),
      loop_closure_cloud_(new ColorPointCloud) {}

LoopClosureDetector::LoopClosureDetector(
    double _loop_closure_searching_distance,
    double _loop_closure_distance_interval,
    double _loop_closure_score_threshold, bool _mono_layer_motion)
    : loop_closure_searching_distance_(_loop_closure_searching_distance),
      loop_closure_distance_interval_(_loop_closure_distance_interval),
      loop_closure_score_threshold_(_loop_closure_score_threshold),
      mono_layer_motion_(_mono_layer_motion),
      kdtree_history_pose_(new pcl::KdTreeFLANN<PointType>()),
      loop_closure_cloud_(new ColorPointCloud) {}

void LoopClosureDetector::SetLidarMapSession(
    const std::shared_ptr<LidarMapSession>& _session) {
  lidar_map_session_ptr_ = _session;
}

LoopClosureResult LoopClosureDetector::DetectLoopClosure(int detect_unit_id) {
  LoopClosureResult loop_closure_result;

  std::vector<LoopCandidate> potential_candidates =
      DetectLoopClosureCandidatesByDistance(detect_unit_id);

  if (potential_candidates.empty()) {
    loop_closure_result.success_flag = false;
    return loop_closure_result;
  }

  // 优先级排序
  // 安装距离升序排序
  sort(potential_candidates.begin(), potential_candidates.end(),
       [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
         return lhs.distance < rhs.distance;
       });

  int closest_id = potential_candidates.back().key_history;

  float score = 0;
  Eigen::Matrix4d final_matrix;

  while (true) {
    bool icp_success =
        CloudRevalidation(detect_unit_id, closest_id, final_matrix, score);

    if (icp_success) {
      loop_closure_result.success_flag = true;
      loop_closure_result.current_scan_id_ = detect_unit_id;
      loop_closure_result.history_scan_id_ = closest_id;
      loop_closure_result.trans_history_to_current = final_matrix;
      loop_closure_result.score_ = score;

      return loop_closure_result;
    } else {
      break;
    }
  }

  // 如果没找到,则失败
  loop_closure_result.current_scan_id_ = detect_unit_id;
  loop_closure_result.history_scan_id_ = closest_id;
  loop_closure_result.trans_history_to_current = final_matrix;
  loop_closure_result.score_ = score;
  loop_closure_result.success_flag = false;
  return loop_closure_result;
}

LoopClosureResultList LoopClosureDetector::DetectLoopClosureAll(
    int detect_unit_id) {
  LoopClosureResultList loop_closure_result_list;

  std::vector<LoopCandidate> potential_candidates =
      DetectLoopClosureCandidatesByDistance(detect_unit_id);

  if (potential_candidates.empty()) {
    return loop_closure_result_list;
  }

  /// 对所有闭环进行遍历
  for (int i = 0; i < potential_candidates.size(); i++) {
    float score = 0;
    Eigen::Matrix4d final_matrix;
    bool icp_success =
        CloudRevalidation(detect_unit_id, potential_candidates[i].key_history,
                          final_matrix, score);
    if (icp_success) {
      LoopClosureResult lcr;
      lcr.success_flag = true;
      lcr.current_scan_id_ = detect_unit_id;
      lcr.history_scan_id_ = potential_candidates[i].key_history;
      lcr.trans_history_to_current = final_matrix;
      lcr.score_ = score;
      loop_closure_result_list.push_back(lcr);
    }
  }

  return loop_closure_result_list;
}

std::vector<LoopCandidate>
LoopClosureDetector::DetectLoopClosureCandidatesByDistance(
    int cur_key_scan_id) {
  CloudTypePtr temp_cloud =
      lidar_map_session_ptr_->GetMapUnitsTranslationCloud();
  CloudTypePtr key_scan_poses(new CloudType);
  pcl::copyPointCloud(*temp_cloud, *key_scan_poses);

  PointType cur_scan_pose = key_scan_poses->points[cur_key_scan_id];

  // 若平面运动，将闭环检测转化到二维上
  if (mono_layer_motion_) {
    for (int i = 0; i < key_scan_poses->size(); i++) {
      key_scan_poses->points[i].z = 0;
    }
    cur_scan_pose.z = 0;
  }

  std::vector<int> point_search_ind_loop;
  std::vector<float> point_search_sq_dis_loop;

  kdtree_history_pose_->setInputCloud(key_scan_poses);
  kdtree_history_pose_->radiusSearch(
      cur_scan_pose, loop_closure_searching_distance_, point_search_ind_loop,
      point_search_sq_dis_loop);

  std::vector<LoopCandidate> loop_closure_candidates;

  for (int i = 0; i < (int)point_search_ind_loop.size(); i++) {
    int history_map_unit_index = point_search_ind_loop[i];

    // 只寻找历史帧
    if (history_map_unit_index > cur_key_scan_id) {
      continue;
    }

    double history_map_unit_distance =
        lidar_map_session_ptr_->GetMapUnitDistance(history_map_unit_index);
    double cur_map_unit_distance =
        lidar_map_session_ptr_->GetMapUnitDistance(cur_key_scan_id);
    // 距离校验
    if (history_map_unit_distance < 0 || cur_map_unit_distance < 0 ||
        (cur_map_unit_distance - history_map_unit_distance) <
            loop_closure_distance_interval_)
      continue;

    LoopCandidate loop_candidate;
    loop_candidate.key_history = history_map_unit_index;
    loop_candidate.key_cur = cur_key_scan_id;
    loop_candidate.distance = cur_map_unit_distance - history_map_unit_distance;

    Eigen::Matrix4d history_pose =
        lidar_map_session_ptr_->GetMapUnitPose(history_map_unit_index);
    Eigen::Matrix4d base_pose =
        lidar_map_session_ptr_->GetMapUnitPose(cur_key_scan_id);

    loop_candidate.eucliden_distance =
        utils::GetDistanceBetweenPoses(base_pose, history_pose);

    loop_closure_candidates.push_back(loop_candidate);
  }

  return loop_closure_candidates;
}

bool LoopClosureDetector::CloudRevalidation(int _cur_id, int _history_id,
                                            Eigen::Matrix4d& _transform,
                                            float& _score) {
  // 搜索点云
  CloudTypePtr cur_key_scan_cloud(new CloudType);
  Eigen::Matrix4d cur_key_scan_pose;
  lidar_map_session_ptr_->FindNearbyKeyFrames(_cur_id, cur_key_scan_cloud, 0,
                                              cur_key_scan_pose);

  CloudTypePtr history_key_scan_nearby_cloud(new CloudType);
  Eigen::Matrix4d history_key_scan_pose;
  lidar_map_session_ptr_->FindNearbyKeyFrames(
      _history_id, history_key_scan_nearby_cloud, history_key_frame_search_num,
      history_key_scan_pose);

  Eigen::Matrix4d guess_transform_matrix =
      history_key_scan_pose.inverse() * cur_key_scan_pose;
  // if (mono_layer_motion_) {
  //   // 把z轴的平移置为0
  //   guess_transform_matrix(0, 3) = 0;
  // }
#if CLOUD_REVALIDATION_STRATEGY == CRS_ICP
  // ICP配准
  utils::DownsampleCloud(*cur_key_scan_cloud, 0.3);
  utils::DownsampleCloud(*history_key_scan_nearby_cloud, 0.3);
  return ICPMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
                  guess_transform_matrix, _transform, _score);

#elif CLOUD_REVALIDATION_STRATEGY == CRS_NDT
  NDTMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
           guess_transform_matrix, _transform, _score);
  return true;

#elif CLOUD_REVALIDATION_STRATEGY == CRS_NDT_ICP
  // NDT配准
  Eigen::Matrix4d ndt_result_pose;
  float ndt_reg_score;
  NDTMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
           guess_transform_matrix, ndt_result_pose, ndt_reg_score);

  // ICP配准
  utils::DownsampleCloud(*cur_key_scan_cloud, 0.3);
  utils::DownsampleCloud(*history_key_scan_nearby_cloud, 0.3);

  return ICPMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
                  ndt_result_pose, _transform, _score);
#elif CLOUD_REVALIDATION_STRATEGY == CRS_GICP
  return GICPMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
                   guess_transform_matrix, _transform, _score);

#endif
}

bool LoopClosureDetector::CloudRevalidation(CloudTypePtr _target_cloud,
                                            CloudTypePtr _source_cloud,
                                            Eigen::Matrix4d _predict_matrix,
                                            Eigen::Matrix4d& _result_matrix,
                                            float& _score) {
  CloudTypePtr cur_key_scan_cloud(new CloudType);
  CloudTypePtr history_key_scan_nearby_cloud(new CloudType);
  pcl::copyPointCloud(*_target_cloud, *history_key_scan_nearby_cloud);
  pcl::copyPointCloud(*_source_cloud, *cur_key_scan_cloud);
#if CLOUD_REVALIDATION_STRATEGY == CRS_ICP
  // ICP配准
  utils::DownsampleCloud(*cur_key_scan_cloud, 0.3);
  utils::DownsampleCloud(*history_key_scan_nearby_cloud, 0.3);
  return ICPMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
                  _predict_matrix, _result_matrix, _score);

#elif CLOUD_REVALIDATION_STRATEGY == CRS_NDT
  NDTMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud, _predict_matrix,
           _result_matrix, _score);
  return true;

#elif CLOUD_REVALIDATION_STRATEGY == CRS_NDT_ICP
  // NDT配准
  Eigen::Matrix4d ndt_result_pose;
  float ndt_reg_score;
  NDTMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud, _predict_matrix,
           ndt_result_pose, ndt_reg_score);

  // ICP配准
  utils::DownsampleCloud(*cur_key_scan_cloud, 0.3);
  utils::DownsampleCloud(*history_key_scan_nearby_cloud, 0.3);

  return ICPMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
                  ndt_result_pose, _result_matrix, _score);
#elif CLOUD_REVALIDATION_STRATEGY == CRS_GICP
  return GICPMatch(history_key_scan_nearby_cloud, cur_key_scan_cloud,
                   _predict_matrix, _result_matrix, _score);

#endif
}

bool LoopClosureDetector::ICPMatch(CloudTypePtr target_cloud,
                                   CloudTypePtr source_cloud,
                                   Eigen::Matrix4d pre_transf,
                                   Eigen::Matrix4d& _icp_transform,
                                   float& _score) {
  /// ICP 配准
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(150);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputTarget(target_cloud);
  icp.setInputSource(source_cloud);

  CloudType final_cloud;
  icp.align(final_cloud, pre_transf.template cast<float>());

  _score = icp.getFitnessScore();
  _icp_transform = icp.getFinalTransformation().template cast<double>();
  if (icp.hasConverged() == false || _score > loop_closure_score_threshold_) {
    AINFO_F(
        "[LoopClosureDetector] Cloud registration failed !  Score : %f ,  "
        "Coverged : %d ",
        _score, icp.hasConverged());
    return false;
  }
  return true;
}

bool LoopClosureDetector::NDTMatch(CloudTypePtr target_cloud,
                                   CloudTypePtr source_cloud,
                                   Eigen::Matrix4d pre_trans,
                                   Eigen::Matrix4d& final_trans,
                                   float& _score) {
  NdtScanMatcher matcher(1.0, 0.5, 4);
  matcher.SetTargetCloud(target_cloud);
  Eigen::Matrix4f final_transf;
  matcher.Match(source_cloud, pre_trans.template cast<float>(), final_transf);
  final_trans = final_transf.template cast<double>();
  _score = matcher.GetMatchScore();
  return true;
}

bool LoopClosureDetector::GICPMatch(CloudTypePtr target_cloud,
                                    CloudTypePtr source_cloud,
                                    Eigen::Matrix4d pre_trans,
                                    Eigen::Matrix4d& final_trans,
                                    float& _score) {
  GICPScanMatcher matcher(0.1);
  matcher.SetTargetCloud(target_cloud);
  Eigen::Matrix4f final_transf;
  matcher.Match(source_cloud, pre_trans.template cast<float>(), final_transf);
  final_trans = final_transf.template cast<double>();
  _score = matcher.GetMatchScore();

  if (_score > loop_closure_score_threshold_) {
    // AINFO_F("[LoopClosureDetector] Cloud registration failed !  Score : %f ",
    //         _score);
    return false;
  }
  return true;
}

void LoopClosureDetector::DrawLoopClosureCloud(CloudTypePtr _target_cloud,
                                               CloudTypePtr _source_cloud,
                                               Eigen::Matrix4d& _init_pose,
                                               Eigen::Matrix4d& _aligned_pose) {
  loop_closure_cloud_->clear();
  size_t target_cloud_size = _target_cloud->size();
  CloudType init_source_cloud;
  pcl::transformPointCloud(*_source_cloud, init_source_cloud, _init_pose);
  size_t source_cloud_size = init_source_cloud.size();
  CloudType aligned_source_cloud;
  pcl::transformPointCloud(*_source_cloud, aligned_source_cloud, _aligned_pose);
  size_t aligned_cloud_size = aligned_source_cloud.size();

  loop_closure_cloud_->reserve(target_cloud_size + source_cloud_size +
                               aligned_cloud_size);
  for (size_t i = 0; i < target_cloud_size; i++) {
    ColorPoint p;
    p.x = _target_cloud->points[i].x;
    p.y = _target_cloud->points[i].y;
    p.z = _target_cloud->points[i].z;
    p.r = 255;
    p.g = 0;
    p.b = 0;
    loop_closure_cloud_->push_back(p);
  }

  for (size_t i = 0; i < source_cloud_size; i++) {
    ColorPoint p;
    p.x = init_source_cloud.points[i].x;
    p.y = init_source_cloud.points[i].y;
    p.z = init_source_cloud.points[i].z;
    p.r = 0;
    p.g = 255;
    p.b = 0;
    loop_closure_cloud_->push_back(p);
  }

  for (size_t i = 0; i < aligned_cloud_size; i++) {
    ColorPoint p;
    p.x = aligned_source_cloud.points[i].x;
    p.y = aligned_source_cloud.points[i].y;
    p.z = aligned_source_cloud.points[i].z;
    p.r = 0;
    p.g = 0;
    p.b = 255;
    loop_closure_cloud_->push_back(p);
  }
}

}  // namespace multi_sensor_mapping
