#include <dataset_adapter/KaistDatasetAdapter.h>
#include <dataset_adapter/LidarTimestampRecovery.h>
#include <multi_sensor_localization/param/ExtrinsicParams.h>
#include <multi_sensor_localization/utils/UtilsPointcloud.h>

#define DTOR M_PI / 180.0
#define RTOD 180.0 / M_PI

using namespace multi_sensor_localization;
namespace dataset_adapter {

KaistDatasetAdapter::KaistDatasetAdapter(std::string path)
    : data_path_(path),
      use_imu_flag_(false),
      use_left_lidar_flag_(false),
      use_right_lidar_flag_(false),
      use_sick_flag_(false),
      use_wheel_odom_flag_(false),
      use_stereo_flag_(false),
      use_gps_flag_(false) {
  imu_topic_ = "/imu/data";
  left_velodyne_topic_ = "/velodyne_left_points";
  right_velodyne_topic_ = "/velodyne_right_points";
  front_sick_topic_ = "/sick_front_scan";
  back_sick_topic_ = "/sick_back_scan";
  wheel_odom_topic_ = "/wheel_odometry";
  left_stereo_topic_ = "/stereo/left_image";
  right_stereo_topic_ = "/stereo/right_image";
  gps_topic_ = "/gps/fix";
}

void KaistDatasetAdapter::SetSensorEnableFlag(bool use_imu, bool use_left_lidar,
                                              bool use_right_lidar,
                                              bool use_sick,
                                              bool use_wheel_odom,
                                              bool use_stereo, bool use_gps) {
  use_imu_flag_ = use_imu;
  use_left_lidar_flag_ = use_left_lidar;
  use_right_lidar_flag_ = use_right_lidar;
  use_sick_flag_ = use_sick;
  use_wheel_odom_flag_ = use_wheel_odom;
  use_stereo_flag_ = use_stereo;
  use_gps_flag_ = use_gps;
}

void KaistDatasetAdapter::WriteDataToRosbag() {
  boost::filesystem::path p(data_path_);
  std::string bag_path = data_path_ + "/" + p.stem().string() + ".bag";
  LOG(INFO) << "Rosbag path : " << bag_path;

  std::shared_ptr<rosbag::Bag> kaist_bag_ptr(new rosbag::Bag);
  kaist_bag_ptr->open(bag_path, rosbag::bagmode::Write);

  WriteImuDataToBag(data_path_, kaist_bag_ptr);
  WriteVelodyneDataToBag(data_path_, kaist_bag_ptr);
  WriteWheelOdomDataToBag(data_path_, kaist_bag_ptr);
  WriteSickDataToBag(data_path_, kaist_bag_ptr);
  kaist_bag_ptr->close();
  WriteExtrinsicParams(data_path_);
  TransformGroundTruthToTumFormat(data_path_);
  RightVelodyneMergeMap(bag_path);
}

void KaistDatasetAdapter::WriteVelodyneDataToBag(
    std::string path, std::shared_ptr<rosbag::Bag> bag_ptr) {
  if ((!use_left_lidar_flag_) && (!use_right_lidar_flag_)) return;
  // 读取velodyne的时间戳
  std::vector<int64_t> velodyne_left_timestamps;
  std::vector<int64_t> velodyne_right_timestamps;

  std::string velodyne_left_stamp_path =
      path + "/sensor_data/VLP_left_stamp.csv";
  std::string velodyne_right_stamp_path =
      path + "/sensor_data/VLP_right_stamp.csv";
  std::ifstream f(velodyne_left_stamp_path.c_str());
  if (!f.good()) {
    LOG(INFO) << "Please check file path. Input path is wrong";
    return;
  }
  f.close();
  // 读取左激光雷达时间戳
  if (use_left_lidar_flag_) {
    std::ifstream infile;
    infile.open(velodyne_left_stamp_path);
    std::string current_line;

    while (std::getline(infile, current_line)) {
      std::istringstream s(current_line);
      std::string field;
      std::vector<int64_t> vec;

      while (std::getline(s, field, ' ')) {
        if (field.empty()) continue;
        vec.push_back(std::atol(field.c_str()));
      }
      velodyne_left_timestamps.push_back(vec.front());
    }
    if (velodyne_left_timestamps.empty()) {
      LOG(INFO) << "Read left Velodyne timestamp wrong !";
      return;
    }
  }
  // 读取右激光雷达的时间戳
  if (use_right_lidar_flag_) {
    std::ifstream infile;
    infile.open(velodyne_right_stamp_path);
    std::string current_line;

    while (std::getline(infile, current_line)) {
      std::istringstream s(current_line);
      std::string field;
      std::vector<int64_t> vec;

      while (std::getline(s, field, ' ')) {
        if (field.empty()) continue;
        vec.push_back(std::atol(field.c_str()));
      }
      velodyne_right_timestamps.push_back(vec.front());
    }
    if (velodyne_right_timestamps.empty()) {
      LOG(INFO) << "Read right Velodyne timestamp wrong !";
      return;
    }
  }
  LOG(INFO) << "Left Velodyne " << velodyne_left_timestamps.size();
  LOG(INFO) << "Right Velodyne " << velodyne_right_timestamps.size();

  // 读取left velodyne的数据
  LidarTimestampRecovery ltr;
  for (size_t i = 0; i < velodyne_left_timestamps.size(); i++) {
    int64_t timestamp_ns = velodyne_left_timestamps[i];
    std::string current_file_name = path + "/sensor_data/VLP_left" + "/" +
                                    std::to_string(timestamp_ns) + ".bin";

    VelRTPointCloudPtr raw_cloud(new VelRTPointCloud);
    std::ifstream file;
    file.open(current_file_name, std::ios::in | std::ios::binary);
    while (!file.eof()) {
      VelRTPoint point;
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));

      int ring = ComputeVLP16Ring(point);
      if (ring < 0) continue;
      point.ring = ring;
      point.time = 0;

      raw_cloud->push_back(point);
    }
    file.close();
    ltr.SetInputCloud(raw_cloud);
  }

  ltr.CorrectStartAngleOfScan();

  for (size_t i = 0; i < velodyne_left_timestamps.size(); i++) {
    int64_t timestamp_ns = velodyne_left_timestamps[i];
    std::string current_file_name = path + "/sensor_data/VLP_left" + "/" +
                                    std::to_string(timestamp_ns) + ".bin";

    VelRTPointCloudPtr raw_cloud(new VelRTPointCloud);
    std::ifstream file;
    file.open(current_file_name, std::ios::in | std::ios::binary);
    while (!file.eof()) {
      VelRTPoint point;
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));

      int ring = ComputeVLP16Ring(point);
      if (ring < 0) continue;
      point.ring = ring;
      point.time = 0;

      raw_cloud->push_back(point);
    }
    file.close();

    ltr.RecoverTimestamp(i, raw_cloud);

    sensor_msgs::PointCloud2::Ptr recover_msg =
        boost::shared_ptr<sensor_msgs::PointCloud2>(
            new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*raw_cloud, *recover_msg);

    ros::Time bag_time;
    bag_time.fromNSec(timestamp_ns);
    // 因为kaist数据集的LiDAR的时间戳是最后一个packet的时间
    ros::Time sensor_time = bag_time - ros::Duration(0.1);
    recover_msg->header.frame_id = "left_velodyne";
    recover_msg->header.stamp = sensor_time;
    bag_ptr->write(left_velodyne_topic_, bag_time, recover_msg);
  }

  std::cout << "left velodyne done" << std::endl;

  ltr.Reset();

  // 读取right velodyne的数据
  for (size_t i = 0; i < velodyne_right_timestamps.size(); i++) {
    int64_t timestamp_ns = velodyne_right_timestamps[i];
    std::string current_file_name = path + "/sensor_data/VLP_right" + "/" +
                                    std::to_string(timestamp_ns) + ".bin";

    VelRTPointCloudPtr raw_cloud(new VelRTPointCloud);
    std::ifstream file;
    file.open(current_file_name, std::ios::in | std::ios::binary);
    while (!file.eof()) {
      VelRTPoint point;
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));

      int ring = ComputeVLP16Ring(point);
      if (ring < 0) continue;
      point.ring = ring;
      point.time = 0;
      raw_cloud->push_back(point);
    }
    ltr.SetInputCloud(raw_cloud);
    file.close();
  }

  ltr.CorrectStartAngleOfScan();

  for (size_t i = 0; i < velodyne_right_timestamps.size(); i++) {
    int64_t timestamp_ns = velodyne_right_timestamps[i];
    std::string current_file_name = path + "/sensor_data/VLP_right" + "/" +
                                    std::to_string(timestamp_ns) + ".bin";

    VelRTPointCloudPtr raw_cloud(new VelRTPointCloud);
    std::ifstream file;
    file.open(current_file_name, std::ios::in | std::ios::binary);
    while (!file.eof()) {
      VelRTPoint point;
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));

      int ring = ComputeVLP16Ring(point);
      if (ring < 0) continue;
      point.ring = ring;
      point.time = 0;
      raw_cloud->push_back(point);
    }
    file.close();

    ltr.RecoverTimestamp(i, raw_cloud);

    sensor_msgs::PointCloud2::Ptr recover_msg =
        boost::shared_ptr<sensor_msgs::PointCloud2>(
            new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*raw_cloud, *recover_msg);

    ros::Time bag_time;
    bag_time.fromNSec(timestamp_ns);
    // 因为kaist数据集的LiDAR的时间戳是最后一个packet的时间
    ros::Time sensor_time = bag_time - ros::Duration(0.1);
    recover_msg->header.frame_id = "right_velodyne";
    recover_msg->header.stamp = sensor_time;
    bag_ptr->write(right_velodyne_topic_, bag_time, recover_msg);
  }

  LOG(INFO) << "Velodyne data are loaded !";
}

void KaistDatasetAdapter::WriteImuDataToBag(
    std::string path, std::shared_ptr<rosbag::Bag> bag_ptr) {
  if (!use_imu_flag_) return;
  FILE* fp;
  fp = fopen((path + "/sensor_data/xsens_imu.csv").c_str(), "r");
  int cnt = 0;
  int64_t stamp;
  double q_x, q_y, q_z, q_w, x, y, z, g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y,
      m_z;
  while (1) {
    sensor_msgs::Imu::Ptr imu_data =
        boost::shared_ptr<sensor_msgs::Imu>(new sensor_msgs::Imu);
    int length = fscanf(
        fp,
        "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
        &stamp, &q_x, &q_y, &q_z, &q_w, &x, &y, &z, &g_x, &g_y, &g_z, &a_x,
        &a_y, &a_z, &m_x, &m_y, &m_z);
    if (length != 8 && length != 17) break;
    if (length == 8) {
      imu_data->header.stamp.fromNSec(stamp);
      imu_data->header.frame_id = "imu";
      imu_data->orientation.x = q_x;
      imu_data->orientation.y = q_y;
      imu_data->orientation.z = q_z;
      imu_data->orientation.w = q_w;
    } else if (length == 17) {
      imu_data->header.stamp.fromNSec(stamp);
      imu_data->header.frame_id = "imu";
      imu_data->orientation.x = q_x;
      imu_data->orientation.y = q_y;
      imu_data->orientation.z = q_z;
      imu_data->orientation.w = q_w;
      imu_data->angular_velocity.x = g_x;
      imu_data->angular_velocity.y = g_y;
      imu_data->angular_velocity.z = g_z;
      imu_data->linear_acceleration.x = a_x;
      imu_data->linear_acceleration.y = a_y;
      imu_data->linear_acceleration.z = a_z;

      imu_data->orientation_covariance[0] = 3;
      imu_data->orientation_covariance[4] = 3;
      imu_data->orientation_covariance[8] = 3;
      imu_data->angular_velocity_covariance[0] = 3;
      imu_data->angular_velocity_covariance[4] = 3;
      imu_data->angular_velocity_covariance[8] = 3;
      imu_data->linear_acceleration_covariance[0] = 3;
      imu_data->linear_acceleration_covariance[4] = 3;
      imu_data->linear_acceleration_covariance[8] = 3;
    }

    bag_ptr->write(imu_topic_, imu_data->header.stamp, imu_data);
    cnt++;
  }
  fclose(fp);
  LOG(INFO) << "IMU data are loaded : " << cnt;
}

void KaistDatasetAdapter::WriteWheelOdomDataToBag(
    std::string path, std::shared_ptr<rosbag::Bag> bag_ptr) {
  if (!use_wheel_odom_flag_) return;
  int encoder_resolution = 0;
  double encoder_left_diameter = 0;
  double encoder_right_diameter = 0;
  double encoder_wheel_base = 0;
  bool encoder_param_load_flag = false;
  // 读取轮速计参数
  std::ifstream file((path + "/calibration/EncoderParameter.txt").c_str());
  std::string str;
  while (std::getline(file, str)) {
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(" "));
    if (!strs[1].compare("resolution:")) {
      encoder_resolution = std::stoi(strs[2]);
    }
    if (!strs[1].compare("left")) {
      encoder_left_diameter = std::stod(strs[4]);
    }
    if (!strs[1].compare("right")) {
      encoder_right_diameter = std::stod(strs[4]);
    }
    if (!strs[1].compare("wheel")) {
      encoder_wheel_base = std::stod(strs[3]);
    }
  }
  if (encoder_resolution > 0 && encoder_left_diameter > 0 &&
      encoder_right_diameter > 0 && encoder_wheel_base > 0) {
    LOG(INFO) << "Load encoder parameters successful !";
    encoder_param_load_flag = true;
  }

  double encoder_x = 0;
  double encoder_y = 0;
  double encoder_theta = 0;

  FILE* fp;
  // 读取轮速计原始数据
  fp = fopen((path + "/sensor_data/encoder.csv").c_str(), "r");
  int64_t pre_left_count = 0, pre_right_count = 0;
  int64_t left_count, right_count;
  int64_t stamp;
  int64_t current_stamp = 0, prev_stamp = 0;

  while (fscanf(fp, "%ld,%ld,%ld\n", &stamp, &left_count, &right_count) == 3) {
    if (prev_stamp == 0) {
      prev_stamp = stamp - 10000000;
    }
    if (pre_left_count == 0 && pre_right_count == 0) {
      pre_left_count = left_count;
      pre_right_count = right_count;
    }
    current_stamp = stamp;

    // calculate odometry

    if (encoder_param_load_flag) {
      int64_t d_left_cnt = left_count - pre_left_count;
      int64_t d_right_cnt = right_count - pre_right_count;

      double left_distnace = ((double)d_left_cnt / (double)encoder_resolution) *
                             encoder_left_diameter * M_PI;
      double right_distance =
          ((double)d_right_cnt / (double)encoder_resolution) *
          encoder_right_diameter * M_PI;
      double stamp_diff = static_cast<double>(current_stamp - prev_stamp);
      double dx = (left_distnace + right_distance) * 0.5;
      double dy = 0.0;
      double dtheta = (right_distance - left_distnace) / encoder_wheel_base;
      double vx = dx / stamp_diff;
      double vy = dy / stamp_diff;
      double vth = dtheta / stamp_diff;

      double delta_x = (dx * cos(encoder_theta));
      double delta_y = (dx * sin(encoder_theta));
      double delta_th = dtheta;

      encoder_x += delta_x;
      encoder_y += delta_y;
      encoder_theta += delta_th;
      geometry_msgs::Quaternion odom_quat =
          tf::createQuaternionMsgFromYaw(encoder_theta);
      // Odometry message
      nav_msgs::Odometry::Ptr odom =
          boost::shared_ptr<nav_msgs::Odometry>(new nav_msgs::Odometry);

      odom->header.stamp.fromNSec(current_stamp);
      odom->header.frame_id = "map";

      // set the position
      odom->pose.pose.position.x = encoder_x;
      odom->pose.pose.position.y = encoder_y;

      odom->pose.pose.position.z = 0.0;
      odom->pose.pose.orientation = odom_quat;

      // pose covariance (6x6)
      odom->pose.covariance[0] = 1;
      odom->pose.covariance[7] = 1;
      odom->pose.covariance[14] = 1;
      odom->pose.covariance[21] = 1;
      odom->pose.covariance[28] = 1;
      odom->pose.covariance[35] = 1;
      // twist covariance(6x6)
      odom->twist.covariance[0] = 1;
      odom->twist.covariance[7] = 1;
      odom->twist.covariance[14] = 1;
      odom->twist.covariance[21] = 1;
      odom->twist.covariance[28] = 1;
      odom->twist.covariance[35] = 1;

      // set the velocity
      odom->child_frame_id = "base_link";
      odom->twist.twist.linear.x = vx;
      odom->twist.twist.linear.y = vy;
      odom->twist.twist.angular.z = vth;

      bag_ptr->write(wheel_odom_topic_, odom->header.stamp, odom);
    }
    prev_stamp = current_stamp;
    pre_left_count = left_count;
    pre_right_count = right_count;
  }

  LOG(INFO) << "Encoder data are loaded";
  fclose(fp);
}

void KaistDatasetAdapter::WriteSickDataToBag(
    string path, std::shared_ptr<rosbag::Bag> bag_ptr) {
  if (!use_sick_flag_) return;

  std::vector<int64_t> sick_back_timestamps;
  std::vector<int64_t> sick_middle_timestamps;
  std::string sick_back_stamp_path = path + "/sensor_data/SICK_back_stamp.csv";
  std::string sick_middle_stamp_path =
      path + "/sensor_data/SICK_middle_stamp.csv";
  std::ifstream f(sick_back_stamp_path.c_str());
  if (!f.good()) {
    LOG(INFO) << "Please check file path. Input path is wrong";
    return;
  }
  f.close();

  {
    std::ifstream infile;
    infile.open(sick_back_stamp_path);
    std::string current_line;

    while (std::getline(infile, current_line)) {
      std::istringstream s(current_line);
      std::string field;
      std::vector<int64_t> vec;

      while (std::getline(s, field, ' ')) {
        if (field.empty()) continue;
        vec.push_back(std::atol(field.c_str()));
      }
      sick_back_timestamps.push_back(vec.front());
    }
    if (sick_back_timestamps.empty()) {
      LOG(INFO) << "Read Sick back timestamp wrong !";
      return;
    }
  }
  {
    std::ifstream infile;
    infile.open(sick_middle_stamp_path);
    std::string current_line;

    while (std::getline(infile, current_line)) {
      std::istringstream s(current_line);
      std::string field;
      std::vector<int64_t> vec;

      while (std::getline(s, field, ' ')) {
        if (field.empty()) continue;
        vec.push_back(std::atol(field.c_str()));
      }
      sick_middle_timestamps.push_back(vec.front());
    }
    if (sick_middle_timestamps.empty()) {
      LOG(INFO) << "Read Sick middle timestamp wrong !";
      return;
    }
  }

  LOG(INFO) << "back sick : " << sick_back_timestamps.size();
  LOG(INFO) << "middle sick : " << sick_middle_timestamps.size();

  double angle_diff = 0.0116355288774 * RTOD;
  double start_ang = -5.0;

  /// 读取数据并存入到rosbag中
  for (size_t i = 0; i < sick_back_timestamps.size(); i++) {
    int64_t timestamp_ns = sick_back_timestamps[i];
    std::string current_file_name = path + "/sensor_data/SICK_back" + "/" +
                                    to_string(timestamp_ns) + ".bin";

    std::vector<std::pair<float, float>> laser_data;
    std::ifstream file;
    file.open(current_file_name, ios::in | ios::binary);
    while (!file.eof()) {
      float range;
      float intensity;
      file.read(reinterpret_cast<char*>(&range), sizeof(range));
      file.read(reinterpret_cast<char*>(&intensity), sizeof(intensity));
      laser_data.push_back(std::make_pair(range, intensity));
    }
    file.close();

    /// 将距离转化为点云数据
    VelRTPointCloud laser_cloud;
    for (size_t j = 0; j < laser_data.size(); j++) {
      if (laser_data[j].first > 80 || laser_data[j].first < 0.7) continue;
      VelRTPoint point;
      double alpha_deg = start_ang + static_cast<double>(j) * angle_diff;
      point.x = laser_data[j].first * cos(alpha_deg * DTOR);
      point.y = laser_data[j].first * sin(alpha_deg * DTOR);
      point.z = 0.0;
      point.intensity = laser_data[j].second;
      point.ring = 0;
      // 这个时间戳是猜的，手册里没有找到 100Hz 角分辨率是1°
      point.time = 0.01 / 360.0 * j;
      laser_cloud.push_back(point);
    }

    sensor_msgs::PointCloud2::Ptr laser_msg =
        boost::shared_ptr<sensor_msgs::PointCloud2>(
            new sensor_msgs::PointCloud2);
    pcl::toROSMsg(laser_cloud, *laser_msg);
    laser_msg->header.frame_id = "back_sick";
    laser_msg->header.stamp.fromNSec(timestamp_ns);

    bag_ptr->write(back_sick_topic_, laser_msg->header.stamp, laser_msg);
  }

  for (size_t i = 0; i < sick_middle_timestamps.size(); i++) {
    int64_t timestamp_ns = sick_middle_timestamps[i];
    std::string current_file_name = path + "/sensor_data/SICK_middle" + "/" +
                                    to_string(timestamp_ns) + ".bin";

    std::vector<std::pair<float, float>> laser_data;
    std::ifstream file;
    file.open(current_file_name, ios::in | ios::binary);
    while (!file.eof()) {
      float range;
      float intensity;
      file.read(reinterpret_cast<char*>(&range), sizeof(range));
      file.read(reinterpret_cast<char*>(&intensity), sizeof(intensity));
      laser_data.push_back(std::make_pair(range, intensity));
    }
    file.close();

    /// 将距离转化为点云数据
    VelRTPointCloud laser_cloud;
    for (size_t j = 0; j < laser_data.size(); j++) {
      if (laser_data[j].first > 80 || laser_data[j].first < 0.7) continue;
      VelRTPoint point;
      double alpha_deg = start_ang + static_cast<double>(j) * angle_diff;
      point.x = laser_data[j].first * cos(alpha_deg * DTOR);
      point.y = laser_data[j].first * sin(alpha_deg * DTOR);
      point.z = 0.0;
      point.intensity = laser_data[j].second;
      point.ring = 0;
      // 这个时间戳是猜的，手册里没有找到 100Hz 角分辨率是1°
      point.time = 0.01 / 360.0 * j;
      laser_cloud.push_back(point);
    }
    sensor_msgs::PointCloud2::Ptr laser_msg =
        boost::shared_ptr<sensor_msgs::PointCloud2>(
            new sensor_msgs::PointCloud2);
    pcl::toROSMsg(laser_cloud, *laser_msg);
    laser_msg->header.frame_id = "middle_sick";
    laser_msg->header.stamp.fromNSec(timestamp_ns);

    bag_ptr->write(front_sick_topic_, laser_msg->header.stamp, laser_msg);
  }

  LOG(INFO) << "SICK data are loaded !";
}

void KaistDatasetAdapter::WriteExtrinsicParams(std::string path) {
  std::string extrinsic_path = data_path_ + "/extrinsic_param.yaml";

  multi_sensor_localization::ExtrinsicParams extrinsic_param;

  // 读取 IMU外参
  {
    std::ifstream file((path + "/calibration/Vehicle2IMU.txt").c_str());
    std::string str;
    Eigen::Matrix3f vehicle_sensor_R;
    Eigen::Vector3f vehicle_sensor_T;
    bool load_R_success_flag = false;
    bool load_T_success_flag = false;
    while (std::getline(file, str)) {
      std::vector<std::string> strs;
      boost::split(strs, str, boost::is_any_of(" "));
      if (!strs[0].compare("R:")) {
        vehicle_sensor_R(0, 0) = std::stof(strs[1]);
        vehicle_sensor_R(0, 1) = std::stof(strs[2]);
        vehicle_sensor_R(0, 2) = std::stof(strs[3]);
        vehicle_sensor_R(1, 0) = std::stof(strs[4]);
        vehicle_sensor_R(1, 1) = std::stof(strs[5]);
        vehicle_sensor_R(1, 2) = std::stof(strs[6]);
        vehicle_sensor_R(2, 0) = std::stof(strs[7]);
        vehicle_sensor_R(2, 1) = std::stof(strs[8]);
        vehicle_sensor_R(2, 2) = std::stof(strs[9]);
        load_R_success_flag = true;
      }
      if (!strs[0].compare("T:")) {
        vehicle_sensor_T(0) = std::stof(strs[1]);
        vehicle_sensor_T(1) = std::stof(strs[2]);
        vehicle_sensor_T(2) = std::stof(strs[3]);
        load_T_success_flag = true;
      }
    }
    if (load_R_success_flag && load_T_success_flag) {
      extrinsic_param.baselink_to_imu.block<3, 3>(0, 0) = vehicle_sensor_R;
      extrinsic_param.baselink_to_imu.block<3, 1>(0, 3) = vehicle_sensor_T;
    }
  }

  // 读取left velodyne的外参
  {
    std::ifstream file((path + "/calibration/Vehicle2LeftVLP.txt").c_str());
    std::string str;
    Eigen::Matrix3f vehicle_sensor_R;
    Eigen::Vector3f vehicle_sensor_T;
    bool load_R_success_flag = false;
    bool load_T_success_flag = false;
    while (std::getline(file, str)) {
      std::vector<std::string> strs;
      boost::split(strs, str, boost::is_any_of(" "));
      if (!strs[0].compare("R:")) {
        vehicle_sensor_R(0, 0) = std::stof(strs[1]);
        vehicle_sensor_R(0, 1) = std::stof(strs[2]);
        vehicle_sensor_R(0, 2) = std::stof(strs[3]);
        vehicle_sensor_R(1, 0) = std::stof(strs[4]);
        vehicle_sensor_R(1, 1) = std::stof(strs[5]);
        vehicle_sensor_R(1, 2) = std::stof(strs[6]);
        vehicle_sensor_R(2, 0) = std::stof(strs[7]);
        vehicle_sensor_R(2, 1) = std::stof(strs[8]);
        vehicle_sensor_R(2, 2) = std::stof(strs[9]);
        load_R_success_flag = true;
      }
      if (!strs[0].compare("T:")) {
        vehicle_sensor_T(0) = std::stof(strs[1]);
        vehicle_sensor_T(1) = std::stof(strs[2]);
        vehicle_sensor_T(2) = std::stof(strs[3]);
        load_T_success_flag = true;
      }
    }
    if (load_R_success_flag && load_T_success_flag) {
      extrinsic_param.baselink_to_lidar.block<3, 3>(0, 0) = vehicle_sensor_R;
      extrinsic_param.baselink_to_lidar.block<3, 1>(0, 3) = vehicle_sensor_T;
    }
  }
  // 读取right velodyne的外参
  {
    std::ifstream file((path + "/calibration/Vehicle2RightVLP.txt").c_str());
    std::string str;
    Eigen::Matrix3f vehicle_sensor_R;
    Eigen::Vector3f vehicle_sensor_T;
    bool load_R_success_flag = false;
    bool load_T_success_flag = false;
    while (std::getline(file, str)) {
      std::vector<std::string> strs;
      boost::split(strs, str, boost::is_any_of(" "));
      if (!strs[0].compare("R:")) {
        vehicle_sensor_R(0, 0) = std::stof(strs[1]);
        vehicle_sensor_R(0, 1) = std::stof(strs[2]);
        vehicle_sensor_R(0, 2) = std::stof(strs[3]);
        vehicle_sensor_R(1, 0) = std::stof(strs[4]);
        vehicle_sensor_R(1, 1) = std::stof(strs[5]);
        vehicle_sensor_R(1, 2) = std::stof(strs[6]);
        vehicle_sensor_R(2, 0) = std::stof(strs[7]);
        vehicle_sensor_R(2, 1) = std::stof(strs[8]);
        vehicle_sensor_R(2, 2) = std::stof(strs[9]);
        load_R_success_flag = true;
      }
      if (!strs[0].compare("T:")) {
        vehicle_sensor_T(0) = std::stof(strs[1]);
        vehicle_sensor_T(1) = std::stof(strs[2]);
        vehicle_sensor_T(2) = std::stof(strs[3]);
        load_T_success_flag = true;
      }
    }
    if (load_R_success_flag && load_T_success_flag) {
      extrinsic_param.baselink_to_lidar2.block<3, 3>(0, 0) = vehicle_sensor_R;
      extrinsic_param.baselink_to_lidar2.block<3, 1>(0, 3) = vehicle_sensor_T;

      p_right_vlp_in_vehicle_ = vehicle_sensor_T;
      q_right_vlp_in_vehicle_ = Eigen::Quaternionf(vehicle_sensor_R);
    }
  }

  extrinsic_param.Save(extrinsic_path);

  LOG(INFO) << "Load extrinsic param success ";
}

int KaistDatasetAdapter::ComputeVLP16Ring(const VelRTPoint& raw_point) {
  double lidar_fov_down = -15.0;
  double lidar_fov_resolution = 2.0;

  double depth = sqrt(raw_point.x * raw_point.x + raw_point.y * raw_point.y +
                      raw_point.z * raw_point.z);
  if (depth == 0) return -1;
  double pitch = asin(raw_point.z / depth) / M_PI * 180.0;

  int ring = std::round((pitch - lidar_fov_down) / lidar_fov_resolution);
  if (ring < 0 || ring >= 16) return -1;
  return ring;
}

void KaistDatasetAdapter::TransformGroundTruthToTumFormat(std::string path) {
  std::string file_path = path + "/global_pose.csv";
  std::string save_file_path = path + "/global_pose_tum.txt";
  std::string local_pose_file_path = path + "/local_coordinate_rigin_pose.txt";

  std::ifstream infile;
  infile.open(file_path);
  std::string current_line;
  std::vector<Eigen::VectorXd> info;
  int i = 0;
  while (std::getline(infile, current_line)) {
    std::istringstream s(current_line);
    std::string field;
    std::vector<double> vec;
    while (getline(s, field, ',')) {
      if (field.empty()) continue;
      vec.push_back(atof(field.c_str()));
    }
    Eigen::VectorXd temp(vec.size());
    for (size_t i = 0; i < vec.size(); i++) {
      temp(i) = vec.at(i);
    }

    info.push_back(temp);
  }

  Eigen::Vector3d first_trans, pose_trans;
  std::ofstream outfile, origin_pose_outfile, init_pose_outfile;
  outfile.open(save_file_path);
  origin_pose_outfile.open(local_pose_file_path);
  double x, y, z;

  for (size_t i = 0; i < info.size(); i++) {
    Eigen::VectorXd temp = info.at(i);
    double timestamp = double(temp[0]) * double(1e-9);
    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    pose_matrix(0, 0) = temp[1];
    pose_matrix(0, 1) = temp[2];
    pose_matrix(0, 2) = temp[3];
    pose_matrix(0, 3) = temp[4];
    pose_matrix(1, 0) = temp[5];
    pose_matrix(1, 1) = temp[6];
    pose_matrix(1, 2) = temp[7];
    pose_matrix(1, 3) = temp[8];
    pose_matrix(2, 0) = temp[9];
    pose_matrix(2, 1) = temp[10];
    pose_matrix(2, 2) = temp[11];
    pose_matrix(2, 3) = temp[12];

    if (i == 0) {
      first_trans = pose_matrix.block<3, 1>(0, 3);
      origin_pose_outfile.precision(18);
      origin_pose_outfile << first_trans(0) << " " << first_trans(1) << " "
                          << first_trans(2) << "\n";
      origin_pose_outfile.close();
    }

    pose_trans = (pose_matrix.block<3, 1>(0, 3) - first_trans);
    Eigen::Quaterniond q(pose_matrix.block<3, 3>(0, 0));

    outfile.precision(18);
    outfile << timestamp << " ";
    outfile.precision(5);
    outfile << pose_trans(0) << " " << pose_trans(1) << " " << pose_trans(2)
            << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            << "\n";

    multi_sensor_localization::PoseData lidar_pose_data;
    lidar_pose_data.timestamp = timestamp;
    lidar_pose_data.position =
        q.cast<float>() * p_right_vlp_in_vehicle_ + pose_trans.cast<float>();
    lidar_pose_data.orientation = q.cast<float>() * q_right_vlp_in_vehicle_;
    right_vlp_gt_poses_.push_back(lidar_pose_data);
  }

  outfile.close();
}

void KaistDatasetAdapter::RightVelodyneMergeMap(std::string bag_path) {
  if (right_vlp_gt_poses_.empty()) {
    LOG(INFO) << "please load gt pose first !";
    return;
  }
  std::string lidar_topic = "/velodyne_right_points";
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(lidar_topic);

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

  for (const rosbag::MessageInstance& m : view_) {
    ros::Time rosbag_time = m.getTime();

    if (m.getTopic() == lidar_topic) {
      sensor_msgs::PointCloud2::Ptr lidar_msg =
          m.instantiate<sensor_msgs::PointCloud2>();
      VelRTPointCloudPtr cloud(new VelRTPointCloud);
      pcl::fromROSMsg(*lidar_msg, *cloud);
      double scan_timestamp = lidar_msg->header.stamp.toSec();

      if (right_vlp_gt_poses_.front().timestamp > scan_timestamp) continue;
      last_odom_idx = odom_idx;

      PoseData cur_scan_pose;
      cur_scan_pose.timestamp = scan_timestamp;

      while (odom_idx < right_vlp_gt_poses_.size()) {
        if (right_vlp_gt_poses_[odom_idx - 1].timestamp <= scan_timestamp &&
            right_vlp_gt_poses_[odom_idx].timestamp > scan_timestamp) {
          SlerpPose(right_vlp_gt_poses_[odom_idx - 1],
                    right_vlp_gt_poses_[odom_idx], cur_scan_pose);
          break;
        }
        ++odom_idx;
      }

      if (odom_idx >= right_vlp_gt_poses_.size()) break;

      Eigen::aligned_vector<PoseData> odom_in_Lk_vec;
      odom_in_Lk_vec.reserve(15);

      PoseData T_Lk0_to_Lk0;
      T_Lk0_to_Lk0.timestamp = cur_scan_pose.timestamp;
      T_Lk0_to_Lk0.orientation = Eigen::Quaternionf::Identity();
      T_Lk0_to_Lk0.position = Eigen::Vector3f::Zero();
      odom_in_Lk_vec.push_back(T_Lk0_to_Lk0);  // 当前帧起始点pose

      Eigen::Quaternionf q_inv = cur_scan_pose.orientation.inverse();
      Eigen::Vector3f p_inv = q_inv * (-cur_scan_pose.position);

      for (int i = last_odom_idx - 1; i < right_vlp_gt_poses_.size(); i++) {
        if (right_vlp_gt_poses_[i].timestamp <= cur_scan_pose.timestamp)
          continue;
        if (right_vlp_gt_poses_[i].timestamp > cur_scan_pose.timestamp + 0.11)
          break;

        PoseData T_Lki_to_Lk0;
        T_Lki_to_Lk0.timestamp = right_vlp_gt_poses_[i].timestamp;
        T_Lki_to_Lk0.orientation = q_inv * right_vlp_gt_poses_[i].orientation;
        T_Lki_to_Lk0.position = q_inv * right_vlp_gt_poses_[i].position + p_inv;

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
  }

  /// 保存全局地图
  std::string map_cloud_path = data_path_ + "/global_map_cloud.pcd";
  CloudTypePtr map_cloud_ds(new CloudType);
  utils::DownsampleCloudAdapted(map_cloud, map_cloud_ds, 0.2);
  pcl::io::savePCDFileBinaryCompressed(map_cloud_path, *map_cloud_ds);
}

void KaistDatasetAdapter::SlerpPose(const PoseData& pose0,
                                    const PoseData& pose1, PoseData& pose_out) {
  double scale = (pose_out.timestamp - pose0.timestamp) /
                 (pose1.timestamp - pose0.timestamp);

  assert(scale >= 0 && scale <= 1 && "GetSlerpPose time error");

  pose_out.orientation =
      pose0.orientation.slerp(scale, pose1.orientation);  // 线性球面插值
  pose_out.position =
      pose0.position + scale * (pose1.position - pose0.position);
}

void KaistDatasetAdapter::UndistortedScan(
    const Eigen::aligned_vector<PoseData>& odom_in_Lk_vec, double scan_time,
    VelRTPointCloudPtr raw_cloud, CloudTypePtr output_cloud) {
  for (auto& p : raw_cloud->points) {
    if (pcl_isnan(p.x)) continue;

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
