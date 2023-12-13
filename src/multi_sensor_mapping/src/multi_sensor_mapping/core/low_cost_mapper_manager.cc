#include "multi_sensor_mapping/core/low_cost_mapper_manager.h"

#include "multi_sensor_mapping/core/fast_lio_mapper.h"
#include "multi_sensor_mapping/core/lidar_detector.h"
#include "multi_sensor_mapping/core/tiny_pose_graph_processor.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_2d_visualizer.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

namespace multi_sensor_mapping {

LowCostMapperManager::LowCostMapperManager()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      tag_init_done_flag_(false),
      mapping_status_(ONLINE_ODOMETRY),
      keyframe_update_flag_(false),
      key_frame_cloud_(new CloudType) {}

void LowCostMapperManager::SetParamSetPath(std::string _param_path) {
  param_set_path_ = _param_path;
}

void LowCostMapperManager::SetCachePath(std::string _cache_path) {
  cache_path_ = _cache_path;
}

void LowCostMapperManager::SetPoseIconPath(std::string _icon_path) {
  pose_icon_ = cv::imread(_icon_path, cv::IMREAD_UNCHANGED);
}

void LowCostMapperManager::SetMapResolution(float _resolution) {
  visualizer_->SetMapResolution(_resolution);
}

void LowCostMapperManager::Init() {
  // Step1 加载参数集合
  std::shared_ptr<ParamSet> param_set(new ParamSet);
  if (!param_set->Load(param_set_path_)) {
    return;
  }
  param_set->PrintAll();

  key_pose_threshold_ =
      param_set->GetFastLIOParams()->keyframe_adding_threshold *
      param_set->GetFastLIOParams()->keyframe_adding_threshold;

  // Step2 初始化地图
  lidar_session_ = std::make_shared<LidarMapSession>();

  // Step3 初始化可视化
  visualizer_ = std::make_shared<LidarMapper2DVisualizer>();

  // Step4 初始化里程计
  lidar_mapper_ = std::make_shared<FastLIOMapper>();
  lidar_mapper_->SetParams(param_set);
  lidar_mapper_->Init();

  // Step5 注册建图器回调函数
  lidar_mapper_->RegLidarPoseCallback(std::bind(
      &LowCostMapperManager::PutLidarPose, this, std::placeholders::_1));

  lidar_mapper_->RegFrameCallback(std::bind(&LowCostMapperManager::PutFrame,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));

  // Step6 初始化后端处理器
  pose_graph_processor_ = std::make_shared<TinyPoseGraphProcessor>();
  pose_graph_processor_->SetParams(param_set);
  pose_graph_processor_->Init();

  // Step7 注册后端处理器回调函数
  pose_graph_processor_->RegProcessCallback(std::bind(
      &LowCostMapperManager::PutBackendProgress, this, std::placeholders::_1));
  pose_graph_processor_->RegGlobalCloudCallback(std::bind(
      &LowCostMapperManager::PutGlobalCloud, this, std::placeholders::_1));

#if ENABLE_3D_VISUALIZATION
  pose_graph_processor_->RegPoseGraphInfoCallback(std::bind(
      &LowCostMapperManager::PutPoseGraph, this, std::placeholders::_1));
  pose_graph_processor_->RegLocalMatchInfoCallback(
      std::bind(&LowCostMapperManager::PutLocalMatchInfomation, this,
                std::placeholders::_1));

  // Step8 初始化ROS可视化
  ros_visualizer_ = std::make_shared<LidarMapperRosVisualizer>(param_set);

#endif

  // Step8 初始化激光检测器
  lidar_detector_ = std::make_shared<LidarDetector>();
  lidar_detector_->SetParams(param_set);
  lidar_detector_->Init();

  // Step9 注册激光检测器回调函数
#if ENABLE_LIDAR_TAG_DETECTION
  lidar_detector_->RegTagPoses(std::bind(
      &LowCostMapperManager::PutLidarTagPoses, this, std::placeholders::_1));
#endif

  init_done_flag_ = true;
}

void LowCostMapperManager::Start() {
  if (lidar_mapper_->IsRunning()) {
    AINFO_F(
        "[LowCostMapperManager] LidarMapper process thread have already "
        "running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[LowCostMapperManager] Please init system before start");
    return;
  }

  exit_process_flag_ = false;

  // 开启建图线程
  lidar_mapper_->Start();

#if ENABLE_LIDAR_TAG_DETECTION
  lidar_detector_->Start();
#endif

  lidar_mapping_thread_ =
      std::thread(std::bind(&LowCostMapperManager::LidarMappingProcess, this));
  visualization_thread_ = std ::thread(
      std::bind(&LowCostMapperManager::VisualizationProcess, this));

  start_flag_ = true;
}

void LowCostMapperManager::StartBackend(std::string _session_path) {
  // 加载地图
  if (!lidar_session_->Load(_session_path)) {
    AWARN_F("[LowCostMapperManager] Load session failed");
    return;
  }

  pose_graph_processor_->SetBaseMapSession(lidar_session_);
  pose_graph_processor_->Start();

  visualization_thread_ = std ::thread(
      std::bind(&LowCostMapperManager::VisualizationProcess, this));

  start_flag_ = true;
}

void LowCostMapperManager::StopMapping() {
  AINFO_F("[LowCostMapperManager] Stop Mapping");

  mapping_status_ = OFFLINE_MAPPING;

  // 关闭在线里程计线程
  if (lidar_mapper_->IsRunning()) {
    lidar_mapper_->Stop();
  }

  // 开启后端优化线程
  pose_graph_processor_->SetBaseMapSession(lidar_session_);
  pose_graph_processor_->SetCachePath(cache_path_);
  pose_graph_processor_->Start();
}

void LowCostMapperManager::Stop() {
  if (!start_flag_) {
    return;
  }

  SaveLidarTagPose();

  if (lidar_mapper_->IsRunning()) {
    lidar_mapper_->Stop();
  }

  if (pose_graph_processor_->IsRunning()) {
    pose_graph_processor_->Stop();
  }

#if ENABLE_LIDAR_TAG_DETECTION
  if (lidar_detector_->IsRunning()) {
    lidar_detector_->Stop();
  }
#endif

  exit_process_flag_ = true;

  lidar_mapping_thread_.join();
  visualization_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;

  AINFO_F(
      "[LowCostMapperManager] LowCostMapperManager process has been closed");
}

void LowCostMapperManager::LidarMappingProcess() {
  // 在线建图
  while (!exit_process_flag_ && ros::ok()) {
    if (mapping_status_ == ONLINE_ODOMETRY) {
      // 关键帧判断
      if (!frame_queue_.Empty()) {
        auto frame_info = frame_queue_.Pop();
        KeyframeSelection(frame_info.first, frame_info.second);
      }

#if ENABLE_LIDAR_TAG_DETECTION
      // 激光tag判断
      if (!lidar_tag_queue_.empty()) {
        lidar_tag_mutex_.lock();
        auto lidar_tag_queue_copy = lidar_tag_queue_;
        lidar_tag_mutex_.unlock();

        lidar_pose_mutex_.lock();
        auto lidar_pose_cache_copy = lidar_pose_cache_;
        lidar_pose_mutex_.unlock();

        for (int i = lidar_tag_queue_copy.size() - 1; i >= 0; i--) {
          double frame_timestamp = lidar_tag_queue_copy[i].front().timestamp;
          bool pose_found_flag = false;
          PoseData frame_pose;
          // 查找对应的姿态
          for (int j = lidar_pose_cache_copy.size() - 1; j >= 0; j--) {
            if (fabs(lidar_pose_cache_copy[j].timestamp - frame_timestamp) <
                0.005) {
              frame_pose = lidar_pose_cache_copy[j];
              pose_found_flag = true;
              AINFO_F("< %.6f , %.6f >", frame_timestamp,
                      lidar_pose_cache_copy[j].timestamp);
              break;
            }
          }

          if (pose_found_flag) {
            lidar_tag_pose_.resize(lidar_tag_queue_copy[i].size());
            for (size_t k = 0; k < lidar_tag_queue_copy[i].size(); k++) {
              lidar_tag_pose_[k].orientation =
                  frame_pose.orientation *
                  lidar_tag_queue_copy[i][k].orientation;
              lidar_tag_pose_[k].position =
                  frame_pose.orientation * lidar_tag_queue_copy[i][k].position +
                  frame_pose.position;
            }
            tag_init_done_flag_ = true;
            break;
          }
        }

        if (tag_init_done_flag_) {
          lidar_detector_->Stop();
          lidar_tag_queue_.clear();
          AINFO_F(
              "[LowCostMapperManager] Lidar Tag initialization success, Stop "
              "lidar detector");
        }
      }
#endif
    } else if (mapping_status_ == OFFLINE_MAPPING) {
      /// TODO
    }

    ros::spinOnce();
    usleep(1000);
  }
}

void LowCostMapperManager::VisualizationProcess() {
  /// @brief 可视化图片
  cv::Mat visualization_img;
  /// @brief 姿态更新标志位
  bool update_pose_flag = false;
  /// 姿态像素
  int pose_pix_x;
  int pose_pix_y;
  /// 姿态角度
  float pose_angle;

  while (!exit_process_flag_ && ros::ok()) {
    // 更新点云
    if (keyframe_update_flag_) {
      visualizer_->DisplayCloud(key_frame_cloud_, key_frame_pose_.position,
                                key_frame_pose_.orientation);

#if ENABLE_3D_VISUALIZATION
      CloudTypePtr cloud_in_map_frame(new CloudType());
      pcl::transformPointCloud(*key_frame_cloud_, *cloud_in_map_frame,
                               key_frame_pose_.position,
                               key_frame_pose_.orientation);
      ros_visualizer_->DisplayCurrentCloud(cloud_in_map_frame);
#endif
      keyframe_update_flag_ = false;
    }

    if (!lidar_pose_queue_.Empty()) {
      auto pose = lidar_pose_queue_.PopLatest();
      visualizer_->PoseToPixelCoordinate(pose.position, pose.orientation,
                                         pose_pix_x, pose_pix_y, pose_angle);
#if ENABLE_3D_VISUALIZATION
      Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
      pose_matrix.block<3, 1>(0, 3) = pose.position;
      pose_matrix.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();

      // 发布姿态
      ros_visualizer_->DisplayCurrentPose(pose_matrix);
      ros_visualizer_->DisplayCurrentOdom(pose.timestamp, pose_matrix);
#endif
      update_pose_flag = true;
    }

    if (!global_cloud_queue_.Empty()) {
      visualizer_->DisplayGlobalCloud(global_cloud_queue_.Pop());
    }

    if (visualizer_->GetVisulizationImage(visualization_img)) {
      if (update_pose_flag) {
        cv::Mat color_image;
        cv::cvtColor(visualization_img, color_image, cv::COLOR_GRAY2RGB);
        // cv::Mat img_part =
        //     color_image(cv::Rect(pose_pix_y - 0.5 * pose_icon_.cols,
        //                          pose_pix_x - 0.5 * pose_icon_.rows,
        //                          pose_icon_.cols, pose_icon_.rows));
        // cv::addWeighted(img_part, 0.5, pose_icon_, 0.5, 0.0, img_part);

        cv::circle(color_image, cv::Point(pose_pix_y, pose_pix_x), 5,
                   cv::Scalar(255, 0, 0), cv::FILLED);
        cv::imshow("visualization", color_image);
        cv::waitKey(1);
      } else {
        cv::imshow("visualization", visualization_img);
        cv::waitKey(1);
      }
    }

#if ENABLE_3D_VISUALIZATION
    if (!pose_graph_queue_.Empty()) {
      ros_visualizer_->DisplayPoseGraph(pose_graph_queue_.Pop());
    }

    if (!local_match_info_queue_.Empty()) {
      LocalMatchInformation info = local_match_info_queue_.Pop();
      PublishLocalMatchInfo(info);
    }

    if (tag_init_done_flag_) {
      ros_visualizer_->DisplayTagPoses(lidar_tag_pose_);
    }
#endif

    ros::spinOnce();
    usleep(10000);
  }
}

void LowCostMapperManager::PutLidarPose(const PoseData& _pose_data) {
  lidar_pose_queue_.Push(_pose_data);

#if ENABLE_LIDAR_TAG_DETECTION
  if (!tag_init_done_flag_) {
    lidar_pose_mutex_.lock();
    lidar_pose_cache_.push_back(_pose_data);
    // 缓存20帧数据
    if (lidar_pose_cache_.size() > 20) {
      lidar_pose_cache_.pop_front();
    }
    lidar_pose_mutex_.unlock();
  }
#endif
}

void LowCostMapperManager::PutFrame(const CloudTypePtr& _cloud,
                                    const PoseData& _pose) {
  frame_queue_.Push(std::make_pair(_cloud, _pose));
}

void LowCostMapperManager::PutBackendProgress(const double& _progress) {
  backend_progress_queue_.Push(_progress);
}

void LowCostMapperManager::PutPoseGraph(const PoseGraph& _graph) {
  pose_graph_queue_.Push(_graph);
}

void LowCostMapperManager::PutLocalMatchInfomation(
    const LocalMatchInformation& _local_match_info) {
  local_match_info_queue_.Push(_local_match_info);
}

void LowCostMapperManager::PutGlobalCloud(const CloudTypePtr& _cloud) {
  global_cloud_queue_.Push(_cloud);
}

void LowCostMapperManager::PutLidarTagPoses(
    const std::vector<PoseData>& _tag_poses) {
  if (!_tag_poses.empty()) {
    lidar_tag_mutex_.lock();
    lidar_tag_queue_.push_back(_tag_poses);
    lidar_tag_mutex_.unlock();
  }
}

void LowCostMapperManager::KeyframeSelection(const CloudTypePtr& _cloud,
                                             const PoseData& _pose) {
  if (_cloud->size() < 100) {
    return;
  }
  // 判断关键帧
  if (lidar_session_->GetMapUnitSize() == 0 || CheckKeyScan(_pose)) {
    utils::UniformSampleCloud(*_cloud, *key_frame_cloud_, 0.1);

    // 添加关键帧
    Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
    pose_mat.block<3, 3>(0, 0) = _pose.orientation.toRotationMatrix();
    pose_mat.block<3, 1>(0, 3) = _pose.position;
    std::shared_ptr<LidarMapUnit> map_unit(
        new LidarMapUnit(_pose.timestamp, key_frame_cloud_, pose_mat));
    lidar_session_->AddMapUnit(map_unit);

    key_frame_pose_ = _pose;

    keyframe_update_flag_ = true;

    // AINFO_F("[LowCostMapperManager] Add key frame, pose: %f, %f, %f",
    //         _pose.position(0), _pose.position(1), _pose.position(2));
  }
}

bool LowCostMapperManager::CheckKeyScan(const PoseData& _pose) {
  static Eigen::Vector3d last_key_frame_pose(0, 0, 0);

  double delta_pose_sqr = (_pose.position(0) - last_key_frame_pose(0)) *
                              (_pose.position(0) - last_key_frame_pose(0)) +
                          (_pose.position(1) - last_key_frame_pose(1)) *
                              (_pose.position(1) - last_key_frame_pose(1));

  if (delta_pose_sqr > key_pose_threshold_) {
    last_key_frame_pose = _pose.position;
    return true;
  }
  return false;
}

void LowCostMapperManager::PublishLocalMatchInfo(
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
#if ENABLE_3D_VISUALIZATION
  ros_visualizer_->DisplayLoopClosureCloud(target_cloud_in_local,
                                           source_cloud_in_local);

  ros_visualizer_->DisplaySelectedEdge(
      _local_match_info.source_pose_in_map.block<3, 1>(0, 3),
      _local_match_info.target_pose_in_map.block<3, 1>(0, 3));

#endif
}

void LowCostMapperManager::SaveLidarTagPose() {
  if (!tag_init_done_flag_) {
    return;
  }

  if (!boost::filesystem::exists(cache_path_)) {
    boost::filesystem::create_directories(cache_path_);
  }

  YAML::Node tag_info_node;
  for (auto pose : lidar_tag_pose_) {
    YAML::Node tag_node;
    tag_node["x"] = pose.position(0);
    tag_node["y"] = pose.position(1);
    tag_node["z"] = pose.position(2);

    tag_node["qx"] = pose.orientation.x();
    tag_node["qy"] = pose.orientation.y();
    tag_node["qz"] = pose.orientation.z();
    tag_node["qw"] = pose.orientation.w();
    tag_info_node.push_back(tag_node);
  }

  std::string tag_info_path = cache_path_ + "/tag_info.yaml";
  std::ofstream tag_info_file(tag_info_path.c_str());
  tag_info_file << tag_info_node;
  tag_info_file.close();
}

}  // namespace multi_sensor_mapping