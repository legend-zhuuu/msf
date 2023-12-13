#include <dataset_adapter/FordDatasetAdapter.h>
#include <dataset_adapter/VelodyneTransform.h>
#include <dirent.h>
#include <multi_sensor_localization/utils/UtilsPointcloud.h>

using namespace multi_sensor_localization;

namespace dataset_adapter {

FordDatasetAdapter::FordDatasetAdapter(std::string cache_path)
    : cache_path_(cache_path),
      use_imu_flag_(false),
      use_yellow_lidar_flag_(false),
      use_red_lidar_flag_(false),
      use_blue_lidar_flag_(false),
      use_green_lidar_flag_(false),
      use_gps_flag_(false) {
  imu_topic_ = "/imu";
  yellow_velodyne_topic_ = "/velodyne_yellow_points";
  red_velodyne_topic_ = "/velodyne_red_points";
  blue_velodyne_topic_ = "/velodyne_blue_points";
  green_velodyne_topic_ = "/velodyne_green_points";
  gps_topic_ = "/gps";
}

void FordDatasetAdapter::SetSensorEnableFlag(
    bool use_imu, bool use_yellow_lidar, bool use_red_lidar,
    bool use_blue_lidar, bool use_green_lidar, bool use_gps) {
  use_imu_flag_ = use_imu;
  use_yellow_lidar_flag_ = use_yellow_lidar;
  use_red_lidar_flag_ = use_red_lidar;
  use_blue_lidar_flag_ = use_blue_lidar;
  use_green_lidar_flag_ = use_green_lidar;
  use_gps_flag_ = use_gps;
}

void FordDatasetAdapter::GenerateMap(std::string map_path) {
  std::string cloud_folder_path = map_path + "/3d_point_cloud";
  std::string ground_cloud_folder_path = map_path + "/ground_reflectivity";

  std::vector<std::string> pcd_paths;
  GetFileNames(cloud_folder_path, pcd_paths);
  GetFileNames(ground_cloud_folder_path, pcd_paths);

  if (pcd_paths.empty()) return;

  CloudType map_cloud;
  for (const std::string path : pcd_paths) {
    CloudTypePtr map_part(new CloudType);
    if (pcl::io::loadPCDFile(path.c_str(), *map_part) != -1) {
      CloudTypePtr map_part_filter(new CloudType);
      utils::DownsampleCloudAdapted(map_part, map_part_filter, 0.2);
      map_cloud += *map_part_filter;
    }
  }

  Eigen::Vector4f p_min, p_max;
  pcl::getMinMax3D(map_cloud, p_min, p_max);

  float center_x, center_y;
  center_x = (p_min(0) + p_max(0)) / 2;
  center_y = (p_min(1) + p_max(1)) / 2;

  for (size_t i = 0; i < map_cloud.points.size(); i++) {
    map_cloud.points[i].x -= center_x;
    map_cloud.points[i].y -= center_y;
  }

  std::string map_offset_path = map_path + "/map_offset.txt";
  std::ofstream offset_outfile;
  offset_outfile.open(map_offset_path);
  offset_outfile << "center_x: " << center_x << "\n";
  offset_outfile << "center_y: " << center_y << "\n";
  offset_outfile.close();

  std::string save_map_path = map_path + "/global_cloud.pcd";
  pcl::io::savePCDFileBinaryCompressed(save_map_path, map_cloud);
}

void FordDatasetAdapter::ParseRawBag(string bag_path, std::string lidar_model,
                                     std::string calibration_path,
                                     double min_range, double max_range,
                                     double view_direction, double view_width) {
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    LOG(INFO) << "Wrong bag path";
    return;
  }

  std::string out_bag_path = cache_path_ + "/" + p.stem().string() + "-LI.bag";

  std::string gt_file_path =
      cache_path_ + "/" + p.stem().string() + "_gt_tum.txt";

  std::ofstream gt_outfile;
  gt_outfile.open(gt_file_path);

  rosbag::Bag bag;
  rosbag::Bag bag_write;
  bag.open(bag_path, rosbag::bagmode::Read);
  bag_write.open(out_bag_path, rosbag::bagmode::Write);

  std::string yellow_velodyne_scan_topic = "/lidar_yellow_scan";
  std::string red_velodyne_scan_topic = "/lidar_red_scan";
  std::string blue_velodyne_scan_topic = "/lidar_blue_scan";
  std::string green_velodyne_scan_topic = "/lidar_green_scan";
  std::string ground_truth_pose_topic = "/pose_ground_truth";

  std::map<std::string, VelodyneTransform::Ptr> velodyne_transforms;
  std::vector<std::string> topics;
  topics.push_back(imu_topic_);
  topics.push_back(ground_truth_pose_topic);
  if (use_blue_lidar_flag_) {
    topics.push_back(blue_velodyne_scan_topic);
    VelodyneTransform::Ptr velodyne_transform(new VelodyneTransform(
        lidar_model, "blue_velodyne", calibration_path, min_range, max_range,
        view_direction, view_width));
    velodyne_transforms[blue_velodyne_scan_topic] = velodyne_transform;
  }
  if (use_red_lidar_flag_) {
    topics.push_back(red_velodyne_scan_topic);
    VelodyneTransform::Ptr velodyne_transform(new VelodyneTransform(
        lidar_model, "red_velodyne", calibration_path, min_range, max_range,
        view_direction, view_width));
    velodyne_transforms[red_velodyne_scan_topic] = velodyne_transform;
  }
  if (use_yellow_lidar_flag_) {
    topics.push_back(yellow_velodyne_scan_topic);
    VelodyneTransform::Ptr velodyne_transform(new VelodyneTransform(
        lidar_model, "yellow_velodyne", calibration_path, min_range, max_range,
        view_direction, view_width));
    velodyne_transforms[yellow_velodyne_scan_topic] = velodyne_transform;
  }
  if (use_green_lidar_flag_) {
    topics.push_back(green_velodyne_scan_topic);
    VelodyneTransform::Ptr velodyne_transform(new VelodyneTransform(
        lidar_model, "green_velodyne", calibration_path, min_range, max_range,
        view_direction, view_width));
    velodyne_transforms[green_velodyne_scan_topic] = velodyne_transform;
  }
  if (use_gps_flag_) {
    topics.push_back(gps_topic_);
  }

  rosbag::View view_;
  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  view_.addQuery(bag, rosbag::TopicQuery(topics), time_init,
                 view_full.getEndTime());

  for (const rosbag::MessageInstance& m : view_) {
    ros::Time rosbag_time = m.getTime();
    if (m.getTopic() == yellow_velodyne_scan_topic) {
      velodyne_msgs::VelodyneScan::ConstPtr scan_msg =
          m.instantiate<velodyne_msgs::VelodyneScan>();
      sensor_msgs::PointCloud2::Ptr new_msg =
          boost::shared_ptr<sensor_msgs::PointCloud2>(
              new sensor_msgs::PointCloud2);
      velodyne_transforms[yellow_velodyne_scan_topic]->ProcessScan(scan_msg,
                                                                   new_msg);
      bag_write.write(yellow_velodyne_topic_, rosbag_time, new_msg);
    }
    if (m.getTopic() == red_velodyne_scan_topic) {
      velodyne_msgs::VelodyneScan::ConstPtr scan_msg =
          m.instantiate<velodyne_msgs::VelodyneScan>();
      sensor_msgs::PointCloud2::Ptr new_msg =
          boost::shared_ptr<sensor_msgs::PointCloud2>(
              new sensor_msgs::PointCloud2);
      velodyne_transforms[red_velodyne_scan_topic]->ProcessScan(scan_msg,
                                                                new_msg);
      bag_write.write(red_velodyne_topic_, rosbag_time, new_msg);
    }
    if (m.getTopic() == blue_velodyne_scan_topic) {
      velodyne_msgs::VelodyneScan::ConstPtr scan_msg =
          m.instantiate<velodyne_msgs::VelodyneScan>();
      sensor_msgs::PointCloud2::Ptr new_msg =
          boost::shared_ptr<sensor_msgs::PointCloud2>(
              new sensor_msgs::PointCloud2);
      velodyne_transforms[blue_velodyne_scan_topic]->ProcessScan(scan_msg,
                                                                 new_msg);
      bag_write.write(blue_velodyne_topic_, rosbag_time, new_msg);
    }
    if (m.getTopic() == green_velodyne_scan_topic) {
      velodyne_msgs::VelodyneScan::ConstPtr scan_msg =
          m.instantiate<velodyne_msgs::VelodyneScan>();
      sensor_msgs::PointCloud2::Ptr new_msg =
          boost::shared_ptr<sensor_msgs::PointCloud2>(
              new sensor_msgs::PointCloud2);
      velodyne_transforms[green_velodyne_scan_topic]->ProcessScan(scan_msg,
                                                                  new_msg);
      bag_write.write(green_velodyne_topic_, rosbag_time, new_msg);
    }
    if (m.getTopic() == imu_topic_) {
      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      bag_write.write(imu_topic_, rosbag_time, imu_msg);
    }
    if (m.getTopic() == gps_topic_) {
      sensor_msgs::NavSatFix::ConstPtr gps_msg =
          m.instantiate<sensor_msgs::NavSatFix>();
      bag_write.write(gps_topic_, rosbag_time, gps_msg);
    }
    if (m.getTopic() == ground_truth_pose_topic) {
      geometry_msgs::PoseStamped::ConstPtr gt_pose_msg =
          m.instantiate<geometry_msgs::PoseStamped>();
      static bool first_pose_flag = true;
      static Eigen::Vector3f fisrt_pose;
      static Eigen::Quaternionf first_q_inv;
      if (first_pose_flag) {
        fisrt_pose = Eigen::Vector3f(gt_pose_msg->pose.position.x,
                                     gt_pose_msg->pose.position.y,
                                     gt_pose_msg->pose.position.z);
        first_q_inv = Eigen::Quaternionf(
            gt_pose_msg->pose.orientation.w, gt_pose_msg->pose.orientation.x,
            gt_pose_msg->pose.orientation.y, gt_pose_msg->pose.orientation.z);
        first_q_inv = first_q_inv.inverse();
        first_pose_flag = false;
      }

      multi_sensor_localization::PoseData gt_pose_data;
      gt_pose_data.timestamp = gt_pose_msg->header.stamp.toSec();
      gt_pose_data.position = (Eigen::Vector3f(gt_pose_msg->pose.position.x,
                                               gt_pose_msg->pose.position.y,
                                               gt_pose_msg->pose.position.z) -
                               fisrt_pose);
      gt_pose_data.orientation = Eigen::Quaternionf(
          gt_pose_msg->pose.orientation.w, gt_pose_msg->pose.orientation.x,
          gt_pose_msg->pose.orientation.y, gt_pose_msg->pose.orientation.z);

      gt_outfile.precision(18);
      gt_outfile << gt_pose_data.timestamp << " ";
      gt_outfile.precision(5);
      gt_outfile << gt_pose_data.position(0) << " " << gt_pose_data.position(1)
                 << " " << gt_pose_data.position(2) << " "
                 << gt_pose_data.orientation.x() << " "
                 << gt_pose_data.orientation.y() << " "
                 << gt_pose_data.orientation.z() << " "
                 << gt_pose_data.orientation.w() << "\n";

      /// 计算red激光姿态

      gt_pose_data.position += gt_pose_data.orientation * p_red_vlp_in_vehicle_;
      gt_pose_data.orientation *= q_red_vlp_in_vehicle_;
      red_vlp_gt_poses_.push_back(gt_pose_data);
    }
  }

  bag_write.close();
  gt_outfile.close();
}

void FordDatasetAdapter::WriteExtrinsicParams(string path) {
  multi_sensor_localization::ExtrinsicParams extrinsic_param;
  std::string extrinsic_path = path + "/extrinsic_param.yaml";

  /// 由于ford数据集的body frame是朝下的，定义一个vechile坐标系，z轴朝上
  Eigen::Quaternionf vechile_body;

  vechile_body = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
  {
    /// V2 calibration
    Eigen::Quaternionf imu_q(0.999999, 0, 0.000118, -0.000101);
    //    imu_q = vechile_body * imu_q;
    Eigen::Vector3f imu_t(-0.0998, 0.0178, -0.4023);
    //    imu_t = vechile_body * imu_t;

    Eigen::Quaternionf lidar_q;
    Eigen::Vector3f lidar_t;
    if (use_yellow_lidar_flag_) {
      lidar_q = Eigen::Quaternionf(-0.151932, -0.686868, -0.693733, 0.154474);
      lidar_t = Eigen::Vector3f(1.12584, -0.501281, -1.27249);
    } else if (use_green_lidar_flag_) {
      lidar_q = Eigen::Quaternionf(0.158597, -0.688930, -0.691762, -0.147268);
      lidar_t = Eigen::Vector3f(1.12224, 0.520558, -1.29238);
    } else if (use_red_lidar_flag_) {
      lidar_q = Eigen::Quaternionf(0.003582, -0.704648, -0.709546, 0.001830);
      lidar_t = Eigen::Vector3f(1.13216, -0.378115, -1.40641);

      p_red_vlp_in_vehicle_ = lidar_t;
      q_red_vlp_in_vehicle_ = lidar_q;

    } else if (use_blue_lidar_flag_) {
      lidar_q = Eigen::Quaternionf(0.003382, -0.705243, -0.708949, 0.003492);
      lidar_t = Eigen::Vector3f(1.11228, 0.369431, -1.41456);
    }

    //    lidar_q = vechile_body * lidar_q;
    //    lidar_t = vechile_body * lidar_t;

    extrinsic_param.baselink_to_imu.block<3, 3>(0, 0) =
        imu_q.toRotationMatrix();
    extrinsic_param.baselink_to_imu.block<3, 1>(0, 3) = imu_t;

    extrinsic_param.baselink_to_lidar.block<3, 3>(0, 0) =
        lidar_q.toRotationMatrix();
    extrinsic_param.baselink_to_lidar.block<3, 1>(0, 3) = lidar_t;
  }

  extrinsic_param.Save(extrinsic_path);

  LOG(INFO) << "Load extrinsic param success ";
}

bool FordDatasetAdapter::GetFileNames(std::string folder_path,
                                      std::vector<std::string>& pcd_paths) {
  DIR* dirp = opendir(folder_path.c_str());
  struct dirent* dp;
  if (dirp != NULL) {
    while ((dp = readdir(dirp)) != NULL) {
      // only add pcd files
      std::size_t type = std::string(dp->d_name).find(".pcd");
      if (type != std::string::npos) {
        pcd_paths.push_back(folder_path + "/" + std::string(dp->d_name));
      }
    }
    closedir(dirp);
  } else {
    LOG(INFO) << "Can not open folder : " << folder_path;
    return false;
  }
  return true;
}

void FordDatasetAdapter::RedVelodyneMergeMap(string bag_path) {
  if (red_vlp_gt_poses_.empty()) {
    LOG(INFO) << "please load gt pose first !";
    return;
  }

  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    LOG(INFO) << "Wrong bag path";
    return;
  }

  std::string out_bag_path = cache_path_ + "/" + p.stem().string() + "-LI.bag";
  std::string out_map_path = cache_path_ + "/" + p.stem().string() + "-map.pcd";

  rosbag::Bag bag;
  bag.open(out_bag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(red_velodyne_topic_);

  rosbag::View view_;
  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  view_.addQuery(bag, rosbag::TopicQuery(topics), time_init,
                 view_full.getEndTime());

  int odom_idx = 1;
  int last_odom_idx = 0;

  pcl::VoxelGrid<PointType> cloud_filter;
  cloud_filter.setLeafSize(0.2, 0.2, 0.2);

  CloudTypePtr map_cloud(new CloudType);

  int cnt = 0;
  for (const rosbag::MessageInstance& m : view_) {
    ros::Time rosbag_time = m.getTime();

    if (m.getTopic() == red_velodyne_topic_) {
      if (cnt % 5 == 0) {
        sensor_msgs::PointCloud2::Ptr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        VelRTPointCloudPtr cloud(new VelRTPointCloud);
        pcl::fromROSMsg(*lidar_msg, *cloud);
        double scan_timestamp = lidar_msg->header.stamp.toSec();

        if (red_vlp_gt_poses_.front().timestamp > scan_timestamp) continue;
        last_odom_idx = odom_idx;

        PoseData cur_scan_pose;
        cur_scan_pose.timestamp = scan_timestamp;

        while (odom_idx < red_vlp_gt_poses_.size()) {
          if (red_vlp_gt_poses_[odom_idx - 1].timestamp <= scan_timestamp &&
              red_vlp_gt_poses_[odom_idx].timestamp > scan_timestamp) {
            SlerpPose(red_vlp_gt_poses_[odom_idx - 1],
                      red_vlp_gt_poses_[odom_idx], cur_scan_pose);
            break;
          }
          ++odom_idx;
        }

        if (odom_idx >= red_vlp_gt_poses_.size()) break;

        Eigen::aligned_vector<PoseData> odom_in_Lk_vec;
        odom_in_Lk_vec.reserve(15);

        PoseData T_Lk0_to_Lk0;
        T_Lk0_to_Lk0.timestamp = cur_scan_pose.timestamp;
        T_Lk0_to_Lk0.orientation = Eigen::Quaternionf::Identity();
        T_Lk0_to_Lk0.position = Eigen::Vector3f::Zero();
        odom_in_Lk_vec.push_back(T_Lk0_to_Lk0);  // 当前帧起始点pose

        Eigen::Quaternionf q_inv = cur_scan_pose.orientation.inverse();
        Eigen::Vector3f p_inv = q_inv * (-cur_scan_pose.position);

        for (int i = last_odom_idx - 1; i < red_vlp_gt_poses_.size(); i++) {
          if (red_vlp_gt_poses_[i].timestamp <= cur_scan_pose.timestamp)
            continue;
          if (red_vlp_gt_poses_[i].timestamp > cur_scan_pose.timestamp + 0.11)
            break;

          PoseData T_Lki_to_Lk0;
          T_Lki_to_Lk0.timestamp = red_vlp_gt_poses_[i].timestamp;
          T_Lki_to_Lk0.orientation = q_inv * red_vlp_gt_poses_[i].orientation;
          T_Lki_to_Lk0.position = q_inv * red_vlp_gt_poses_[i].position + p_inv;

          odom_in_Lk_vec.push_back(T_Lki_to_Lk0);
        }

        CloudTypePtr undistored_cloud(new CloudType);
        UndistortedScan(odom_in_Lk_vec, cur_scan_pose.timestamp, cloud,
                        undistored_cloud);

        cloud_filter.setInputCloud(undistored_cloud);
        CloudTypePtr undistored_cloud_ds(new CloudType);
        cloud_filter.filter(*undistored_cloud_ds);

        CloudType undistored_cloud_ds_map;
        pcl::transformPointCloud(*undistored_cloud_ds, undistored_cloud_ds_map,
                                 cur_scan_pose.position,
                                 cur_scan_pose.orientation);
        *map_cloud += undistored_cloud_ds_map;
      }

      cnt++;
    }
  }

  /// 保存全局地图
  CloudTypePtr map_cloud_ds(new CloudType);
  utils::DownsampleCloudAdapted(map_cloud, map_cloud_ds, 0.2);
  pcl::io::savePCDFileBinaryCompressed(out_map_path, *map_cloud_ds);
}

void FordDatasetAdapter::SlerpPose(const PoseData& pose0, const PoseData& pose1,
                                   PoseData& pose_out) {
  double scale = (pose_out.timestamp - pose0.timestamp) /
                 (pose1.timestamp - pose0.timestamp);

  assert(scale >= 0 && scale <= 1 && "GetSlerpPose time error");

  pose_out.orientation =
      pose0.orientation.slerp(scale, pose1.orientation);  // 线性球面插值
  pose_out.position =
      pose0.position + scale * (pose1.position - pose0.position);
}

void FordDatasetAdapter::UndistortedScan(
    const Eigen::aligned_vector<PoseData>& odom_in_Lk_vec, double scan_time,
    VelRTPointCloudPtr raw_cloud, CloudTypePtr output_cloud) {
  for (auto& p : raw_cloud->points) {
    if (pcl_isnan(p.x)) continue;
    double range = PointDistance(p);
    if (range < 2 || range > 50) continue;

    PoseData pose_now;
    pose_now.timestamp = scan_time + p.time;

    for (int i = 1; i < odom_in_Lk_vec.size(); ++i) {
      if (odom_in_Lk_vec[i - 1].timestamp <= pose_now.timestamp &&
          pose_now.timestamp <= odom_in_Lk_vec[i].timestamp) {
        SlerpPose(odom_in_Lk_vec[i - 1], odom_in_Lk_vec[i], pose_now);

        Eigen::Vector3f p_in = Eigen::Vector3f(p.x, p.y, p.z);
        Eigen::Vector3f p_out = pose_now.orientation * p_in + pose_now.position;
        PointType point_out;
        point_out.x = p_out[0];
        point_out.y = p_out[1];
        point_out.z = p_out[2];

        point_out.intensity = p.intensity;

        output_cloud->push_back(point_out);
        break;
      }
    }
  }
}

}  // namespace dataset_adapter
