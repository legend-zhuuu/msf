#include "multi_sensor_mapping/core/low_cost_locator_manager.h"

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/core/fast_lio_mapper.h"
#include "multi_sensor_mapping/core/lidar_detector.h"
#include "multi_sensor_mapping/core/tiny_lidar_locator.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/tiny_location_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_2d_visualizer.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

namespace multi_sensor_mapping {

LowCostLocatorManager::LowCostLocatorManager()
    : exit_process_flag_(false),
      init_done_flag_(false),
      start_flag_(false),
      tag_init_done_flag_(false),
      record_pose_flag_(false),
      global_registration_interval_(1.0),
      auto_start_flag_(false),
      global_map_(new CloudType) {}

void LowCostLocatorManager::SetParamSetPath(std::string _param_path) {
  param_set_path_ = _param_path;
}

void LowCostLocatorManager::SetMapPath(std::string _map_path) {
  std::string map_cloud_path = _map_path;
  boost::filesystem::path map_path(_map_path);
  if (map_path.extension() != ".pcd") {
    map_cloud_path = _map_path + "/global_cloud.pcd";
    map_path_ = _map_path;
  } else {
    map_path_ = map_path.parent_path().string();
  }

  // 加载点云地图
  if (pcl::io::loadPCDFile(map_cloud_path, *global_map_) == -1) {
    AWARN_F("[LowCostLocatorManager] Map Cloud Path is wrong : %s",
            map_cloud_path.c_str());
  }

  // 加载tag姿态
  std::string tag_info_path = map_path_ + "/tag_info.yaml";
  YAML::Node tag_info_node;
  try {
    tag_info_node = YAML::LoadFile(tag_info_path);
  } catch (...) {
    AWARN_F(
        "The format of config file [ %s ] is wrong, Please check (e.g. "
        "indentation).",
        tag_info_path.c_str());
    return;
  }

  for (size_t i = 0; i < tag_info_node.size(); i++) {
    Eigen::Vector3d tag_position(tag_info_node[i]["x"].as<double>(),
                                 tag_info_node[i]["y"].as<double>(),
                                 tag_info_node[i]["z"].as<double>());

    Eigen::Quaterniond tag_rotation(tag_info_node[i]["qw"].as<double>(),
                                    tag_info_node[i]["qx"].as<double>(),
                                    tag_info_node[i]["qy"].as<double>(),
                                    tag_info_node[i]["qz"].as<double>());
    Eigen::Matrix4d tag_pose_mat = Eigen::Matrix4d::Identity();
    tag_pose_mat.block<3, 1>(0, 3) = tag_position;
    tag_pose_mat.block<3, 3>(0, 0) = tag_rotation.toRotationMatrix();
    ref_lidar_tag_pose_.push_back(tag_pose_mat);
  }

  if (!ref_lidar_tag_pose_.empty()) {
    tag_init_done_flag_ = true;
    AINFO_F("[LowCostLocatorManager] Load Tag num < %i >",
            (int)ref_lidar_tag_pose_.size());
  }
}

void LowCostLocatorManager::SetMapResolution(float _resolution) {
#if ENABLE_2D_VISUALIZATION
  visualizer_->SetMapResolution(_resolution);
#else
  AINFO_F("[LowCostLocatorManager] 2D visualization is not enabled");
#endif
}

void LowCostLocatorManager::SetRecordPath(std::string _record_path) {
  record_pose_flag_ = true;
  record_cache_path_ = _record_path;
}

void LowCostLocatorManager::Init() {
  // Step1 加载参数集合
  std::shared_ptr<ParamSet> param_set(new ParamSet);
  if (!param_set->Load(param_set_path_)) {
    AWARN_F("[LowCostLocatorManager] Load param set faild , %s",
            param_set_path_.c_str());
    return;
  }
  param_set->PrintAll();

  global_registration_interval_ =
      param_set->GetTinyLocationParams()->registration_interval;
  auto_start_flag_ = param_set->GetTinyLocationParams()->auto_start;

  // Step2 初始化里程计
  lidar_mapper_ = std::make_shared<FastLIOMapper>();
  lidar_mapper_->SetParams(param_set);
  lidar_mapper_->Init();

  lidar_mapper_->RegLidarPoseCallback(std::bind(
      &LowCostLocatorManager::PutLidarPose, this, std::placeholders::_1));
  lidar_mapper_->RegFrameCallback(std::bind(&LowCostLocatorManager::PutFrame,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));

  // Step3 初始化定位器
  lidar_locator_ = std::make_shared<TinyLidarLocator>();
  lidar_locator_->SetParams(param_set);
  lidar_locator_->Init();

  // 设置地图
  if (!global_map_->empty()) {
    lidar_locator_->SetMapCloud(global_map_);
  } else {
    AWARN_F("[LowCostLocatorManager] global map is empty ");
  }

  lidar_locator_->RegLocationPose(std::bind(
      &LowCostLocatorManager::PutLocationPose, this, std::placeholders::_1));
  lidar_locator_->RegLocationCloud(std::bind(
      &LowCostLocatorManager::PutLocationCloud, this, std::placeholders::_1));

  // Step4 初始化tag检测器
  lidar_detector_ = std::make_shared<LidarDetector>();
  lidar_detector_->SetParams(param_set);
  lidar_detector_->Init();

  if (auto_start_flag_) {
    lidar_detector_->RegTagPoses(std::bind(
        &LowCostLocatorManager::PutLidarTagPoses, this, std::placeholders::_1));
  }

// Step5 初始化可视化
#if ENABLE_2D_VISUALIZATION
  visualizer_ = std::make_shared<LidarMapper2DVisualizer>();
#endif

#if ENABLE_3D_VISUALIZATION
  ros_visualizer_ = std::make_shared<LidarMapperRosVisualizer>(param_set);
#endif

  // Step6 初始化位姿订阅器
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  sub_initial_pose_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, &LowCostLocatorManager::InitPoseHandler, this,
      ros::TransportHints().tcpNoDelay());

  init_done_flag_ = true;
}

void LowCostLocatorManager::Start() {
  if (lidar_mapper_->IsRunning()) {
    AINFO_F(
        "[LowCostLocatorManager] LidarMapper process thread have already "
        "running");
    return;
  }

  if (lidar_locator_->IsRunning()) {
    AINFO_F(
        "[LowCostLocatorManager] LidarLocator process thread have already "
        "running");
    return;
  }

  exit_process_flag_ = false;

  // 开启建图线程
  lidar_mapper_->Start();

  // 开启定位线程
  lidar_locator_->Start();

  // 开启激光检测
  if (auto_start_flag_) {
    lidar_detector_->Start();
  }

  location_thread_ =
      std::thread(std::bind(&LowCostLocatorManager::LocationProcess, this));
  visualization_thread_ = std ::thread(
      std::bind(&LowCostLocatorManager::VisualizationProcess, this));

  start_flag_ = true;
}

void LowCostLocatorManager::Stop() {
  if (!start_flag_) {
    return;
  }

  if (lidar_mapper_->IsRunning()) {
    lidar_mapper_->Stop();
  }

  if (lidar_locator_->IsRunning()) {
    lidar_locator_->Stop();
  }

  if (lidar_detector_->IsRunning()) {
    lidar_detector_->Stop();
  }

  exit_process_flag_ = true;

  // 保存缓存数据
  SaveCacheData();

  location_thread_.join();
  visualization_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;

  AINFO_F(
      "[LowCostLocatorManager] LowCostLocatorManager process has been closed");
}

void LowCostLocatorManager::LocationProcess() {
  while (!exit_process_flag_) {
    // 处理激光帧
    if (!frame_queue_.Empty()) {
      auto frame_info = frame_queue_.PopLatest();
      lidar_locator_->InputKeyFrame(frame_info.first, frame_info.second);
    }

    // 处理激光里程计数据
    if (!lidar_pose_queue_.Empty()) {
      auto lio_pose = lidar_pose_queue_.PopLatest();
      lidar_locator_->InputLIOPose(lio_pose);
    }

    // 处理激光tag
    if (!lidar_tag_queue_.Empty()) {
      auto latest_tag_poses = lidar_tag_queue_.PopLatest();
      if (!latest_tag_poses.empty() && !ref_lidar_tag_pose_.empty()) {
        Eigen::Matrix4d detected_tag_pose = Eigen::Matrix4d::Identity();
        detected_tag_pose.block<3, 1>(0, 3) = latest_tag_poses.front().position;
        detected_tag_pose.block<3, 3>(0, 0) =
            latest_tag_poses.front().orientation.toRotationMatrix();
        Eigen::Matrix4d init_frame_pose_mat =
            ref_lidar_tag_pose_.front() * detected_tag_pose.inverse();

        Eigen::Vector3d init_translation =
            init_frame_pose_mat.block<3, 1>(0, 3);
        Eigen::Quaterniond init_quater(init_frame_pose_mat.block<3, 3>(0, 0));
        PoseData init_pose;
        init_pose.position = init_translation;
        init_pose.orientation = init_quater;

        lidar_locator_->InputInitialPose(init_pose);

        AINFO_F(
            "[LowCostLocatorManager] Set initial pose from topic [ ( %f, "
            "%f, %f ) (%f, %f, %f, %f)]",
            init_translation(0), init_translation(1), init_translation(2),
            init_quater.w(), init_quater.x(), init_quater.y(), init_quater.z());

        // 停止检测
        lidar_detector_->Stop();
      }
    }
    usleep(1000);
    ros::spinOnce();
  }
}

void LowCostLocatorManager::VisualizationProcess() {
  double loop_time;
  // 全局点云发布时间
  double pub_global_map_timestamp = 0;

  /// @brief 可视化图片
  cv::Mat visualization_img;
  /// @brief 姿态更新标志位
  bool update_pose_flag = false;
  /// 姿态像素
  int pose_pix_x;
  int pose_pix_y;
  /// 姿态角度
  float pose_angle;

#if ENABLE_2D_VISUALIZATION
  visualizer_->DisplayGlobalCloud(global_map_);
#endif

  while (!exit_process_flag_) {
#if ENABLE_3D_VISUALIZATION
    loop_time = GetSystemTimeSecond();
    if (loop_time - pub_global_map_timestamp > 10.0) {
      pub_global_map_timestamp = loop_time;
      // 全局点云发布
      ros_visualizer_->DisplayGlobalCloud(global_map_);
    }
#endif

    // 定位姿态处理
    if (!location_pose_queue_.Empty()) {
      auto location_pose = location_pose_queue_.PopLatest();
      if (record_pose_flag_) {
        location_pose_cache_container_.push_back(location_pose);
      }
#if ENABLE_3D_VISUALIZATION
      Eigen::Matrix4d location_pose_mat = Eigen::Matrix4d::Identity();
      location_pose_mat.block<3, 3>(0, 0) =
          location_pose.orientation.toRotationMatrix();
      location_pose_mat.block<3, 1>(0, 3) = location_pose.position;
      ros_visualizer_->DisplayCurrentPose(location_pose_mat);
#endif

#if ENABLE_2D_VISUALIZATION
      visualizer_->RefreshGlobalMap(location_pose.position);

      visualizer_->PoseToPixelCoordinate(location_pose.position,
                                         location_pose.orientation, pose_pix_x,
                                         pose_pix_y, pose_angle);
#endif

      update_pose_flag = true;
    }

    // 定位点云显示
    if (!location_cloud_queue_.Empty()) {
      auto location_cloud = location_cloud_queue_.PopLatest();

#if ENABLE_3D_VISUALIZATION
      ros_visualizer_->DisplayCurrentCloud(location_cloud);
#endif
    }

#if ENABLE_2D_VISUALIZATION
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
#endif

    usleep(100000);
  }
}

void LowCostLocatorManager::PutLidarPose(const PoseData& _pose_data) {
  lidar_pose_queue_.Push(_pose_data);
}

void LowCostLocatorManager::PutFrame(const CloudTypePtr& _cloud,
                                     const PoseData& _pose) {
  static double last_key_frame_timestamp = 0;

  if (_cloud->size() < 100) {
    return;
  }

  if (_pose.timestamp - last_key_frame_timestamp >
      global_registration_interval_) {
    frame_queue_.Push(std::make_pair(_cloud, _pose));
    last_key_frame_timestamp = _pose.timestamp;
  }
}

void LowCostLocatorManager::PutLocationPose(const PoseData& _pose_data) {
  location_pose_queue_.Push(_pose_data);
}

void LowCostLocatorManager::PutLocationCloud(const CloudTypePtr& _cloud) {
  location_cloud_queue_.Push(_cloud);
}

void LowCostLocatorManager::PutLidarTagPoses(
    const std::vector<PoseData>& _tag_poses) {
  if (!_tag_poses.empty()) {
    lidar_tag_queue_.Push(_tag_poses);
  }
}

void LowCostLocatorManager::InitPoseHandler(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg) {
  Eigen::Vector3d init_trans = Eigen::Vector3d(_msg->pose.pose.position.x,
                                               _msg->pose.pose.position.y, 0);
  Eigen::Quaterniond init_quater(
      _msg->pose.pose.orientation.w, _msg->pose.pose.orientation.x,
      _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z);

  PoseData init_pose;
  init_pose.position = init_trans;
  init_pose.orientation = init_quater;

  lidar_locator_->InputInitialPose(init_pose);

  AINFO_F(
      "[LowCostLocatorManager] Receive initial pose from topic [ ( %f, %f, %f "
      ") (%f, "
      "%f, %f, %f)]",
      init_trans(0), init_trans(1), init_trans(2), init_quater.w(),
      init_quater.x(), init_quater.y(), init_quater.z());
}

void LowCostLocatorManager::SaveCacheData() {
  if (!record_pose_flag_) {
    return;
  }

  if (!boost::filesystem::exists(record_cache_path_)) {
    boost::filesystem::create_directories(record_cache_path_);
  }

  // 保存缓存轨迹
  {
    std::string location_pose_path = record_cache_path_ + "/location_pose.txt";
    std::ofstream outfile;
    outfile.open(location_pose_path);

    outfile.setf(std::ios::fixed);
    for (auto pose : location_pose_cache_container_) {
      outfile.precision(9);
      outfile << pose.timestamp << " ";
      outfile.precision(5);
      outfile << pose.position(0) << " " << pose.position(1) << " "
              << pose.position(2) << " " << pose.orientation.x() << " "
              << pose.orientation.y() << " " << pose.orientation.z() << " "
              << pose.orientation.w() << "\n";
    }
    outfile.close();
  }

  AINFO_F("[LowCostLocatorManager] Save data to %s ",
          record_cache_path_.c_str());
}

}  // namespace multi_sensor_mapping