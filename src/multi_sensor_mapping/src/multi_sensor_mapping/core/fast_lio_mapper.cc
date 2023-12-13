#include "multi_sensor_mapping/core/fast_lio_mapper.h"

#include "multi_sensor_mapping/core/fast_lio_core.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/watch_dog.h"

namespace multi_sensor_mapping {

FastLIOMapper::FastLIOMapper()
    : LidarMapperBase(),
      sensor_disconnected_flag_(false),
      last_imu_timestamp_(0),
      lidar_mean_scan_time_(0),
      scan_num_(0) {}

void FastLIOMapper::SetParams(const std::shared_ptr<ParamSet>& _param_set) {
  LidarMapperBase::SetParams(_param_set);
  lio_params_ = _param_set->GetFastLIOParams();
}

void FastLIOMapper::Init() {
  AINFO_F("[FastLIOMapper] Init Fast-LIO Mapper");

  // 检测参数
  if (!lio_params_ || !sensor_params_ || !extrinsic_params_) {
    AWARN_F("[FastLIOMapper] No input Params");
    RunException(MSMError(CODE_NO_INPUT_PARAMS));
    return;
  }

  // 初始化Fast-LIO核心
  fast_lio_core_ = std::make_shared<FastLIOCore>();
  fast_lio_core_->SetParams(extrinsic_params_, lio_params_);
  fast_lio_core_->InitializeAll();

  data_converter_ = std::make_shared<DataConverter>(
      lio_params_->lidar_min_distance, lio_params_->lidar_max_distance);

  // 初始化看门狗
  double lidar_time_threshold = 0.5;
  double imu_time_threshold = 0.1;
  lidar_watch_dog_ = WatchDog::Ptr(new WatchDog(lidar_time_threshold));
  imu_watch_dog_ = WatchDog::Ptr(new WatchDog(imu_time_threshold));

  sensor_disconnected_flag_ = false;

  // 注册传感器订阅器
  RegisterSensorSub();

  init_done_flag_ = true;
}

void FastLIOMapper::Start() {
  if (start_flag_) {
    AINFO_F("[FastLIOMapper] Mapping process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[FastLIOMapper] Please init system before start");
    RunException(MSMError(CODE_START_BEFORE_INIT));
    return;
  }

  AINFO_F("[FastLIOMapper] Start Fast-LIO Mapper");

  exit_process_flag_ = false;

  // 主处理线程
  process_thread_ = std::thread(std::bind(&FastLIOMapper::LoopProcess, this));

  start_flag_ = true;
}

void FastLIOMapper::Stop() {
  if (!start_flag_) {
    return;
  }

  exit_process_flag_ = true;
  process_thread_.join();

  AINFO_F("[FastLIOMapper] Clear sub cache");
  int cnt = 0;
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (cnt++ > 20) {
      break;
    }
  }

  start_flag_ = false;

  ResetMapper();

  AINFO_F("[FastLIOMapper] Fast LIO Mapper process thread stopped");
}

void FastLIOMapper::RegisterSensorSub() {
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

  if (this->sensor_params_->imu_param.use_flag) {
    sub_imu_ = nh_->subscribe<sensor_msgs::Imu>(
        this->sensor_params_->imu_param.topic, 1000, &FastLIOMapper::ImuHandler,
        this, ros::TransportHints().tcpNoDelay());
  }

  std::string lidar_topic = sensor_params_->LIOLidarTopic();
  sub_cloud_ = nh_->subscribe<sensor_msgs::PointCloud2>(
      lidar_topic, 100, &FastLIOMapper::CloudHandler, this,
      ros::TransportHints().tcpNoDelay());
}

void FastLIOMapper::ImuHandler(const sensor_msgs::Imu::ConstPtr& _imu_msg) {
  if (!start_flag_) {
    return;
  }
  if (sensor_disconnected_flag_) {
    return;
  }

  IMUData imu_data = data_converter_->ConvertImuData(_imu_msg);
  double timestamp = imu_data.timestamp;

  if (timestamp < last_imu_timestamp_) {
    AWARN_F("[FastLIOMapper] IMU loop back, clear IMU buffer");
    imu_buffer_.clear();
  }

  imu_watch_dog_->Feed();

  last_imu_timestamp_ = timestamp;
  Lock lock_imu(imu_mutex_);
  imu_buffer_.push_back(imu_data);
}

void FastLIOMapper::CloudHandler(
    const sensor_msgs::PointCloud2::ConstPtr& _cloud_msg) {
  if (!start_flag_) {
    return;
  }

  if (sensor_disconnected_flag_) {
    return;
  }
  TimedCloudData cloud_data;
  if (sensor_params_->LIOLidarFormat() == LidarDataFormat::RSLIDAR) {
    cloud_data = data_converter_->ConvertRsCloudData(_cloud_msg);
  } else if (sensor_params_->LIOLidarFormat() ==
             LidarDataFormat::RSLIDAR_LEGACY) {
    cloud_data = data_converter_->ConvertLegacyRsCloudData(_cloud_msg);
  } else if (sensor_params_->MajorLidarFormat() == LidarDataFormat::LIVOX) {
    cloud_data = data_converter_->ConvertLivoxCloudData(_cloud_msg);
  } else {
    AWARN_F("[FastLIOMapper] Unknow lidar format");
    return;
  }

  if (!cloud_data.valid_flag) {
    return;
  }

  lidar_watch_dog_->Feed();
  VelRTPointCloudPtr feature_cloud(new VelRTPointCloud);
  size_t points_size = cloud_data.cloud->size();
  feature_cloud->reserve(points_size);

  int cnt = 0;
  for (size_t i = 0; i < points_size; i++) {
    if (i % lio_params_->point_filter_num == 0) {
      feature_cloud->push_back(cloud_data.cloud->points[i]);
      cnt++;
    }
  }
  feature_cloud->resize(cnt);
  cloud_data.cloud = feature_cloud;
  Lock lock_lidar(lidar_mutex_);
  cloud_buffer_.push_back(cloud_data);
}

void FastLIOMapper::LoopProcess() {
  while (!exit_process_flag_) {
    // 同步传感器数据
    if (SyncSensorData()) {
      // 里程推算
      fast_lio_core_->OdometryOnceScan(current_cloud_data_, current_imu_data_);

      // 上传当前姿态
      RunPutLidarPose(fast_lio_core_->GetCurrentPose());

      // 上传当前帧信息
      RunPutFrame(fast_lio_core_->GetCurrentCloud(),
                  fast_lio_core_->GetCurrentPose());
    }

    /// 若传感器断连，则退出建图器
    if ((!sensor_disconnected_flag_) && !CheckSensorTrigger()) {
      sensor_disconnected_flag_ = true;
      AWARN_F("[FastLIOMapper] Sensor trigger timeout, stop mapping");
    }

    if (!ros::ok()) break;
    ros::spinOnce();

    usleep(200);
  }
}

bool FastLIOMapper::SyncSensorData() {
  // AINFO_F("[FastLIOMapper] cloud buffer size  < %i > , IMU buffer size < %i
  // >",
  //         (int)cloud_buffer_.size(), (int)imu_buffer_.size());

  if (cloud_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  double lidar_end_timestamp;
  {
    Lock lock_lidar(lidar_mutex_);
    current_cloud_data_ = cloud_buffer_.front();
  }
  if (current_cloud_data_.cloud->size() <= 1) {
    lidar_end_timestamp = current_cloud_data_.timestamp + lidar_mean_scan_time_;
    AWARN_F("[FastLIOMapper] Too few input point cloud");
  } else if (current_cloud_data_.cloud->points.back().time <
             0.5 * lidar_mean_scan_time_) {
    lidar_end_timestamp = current_cloud_data_.timestamp + lidar_mean_scan_time_;
  } else {
    scan_num_++;
    lidar_end_timestamp = current_cloud_data_.timestamp +
                          current_cloud_data_.cloud->points.back().time;
    lidar_mean_scan_time_ += (current_cloud_data_.cloud->points.back().time -
                              lidar_mean_scan_time_) /
                             scan_num_;
  }

  // AINFO_F("[FastLIOMapper] imu buffer < %.6f , %.6f > , %.6f ",
  //         imu_buffer_.front().timestamp, imu_buffer_.back().timestamp,
  //         lidar_end_timestamp);

  if (last_imu_timestamp_ < lidar_end_timestamp) {
    // AWARN_F("[FastLIOMapper] Waiting for imu, < %.3f, %.3f >",
    //         last_imu_timestamp_, lidar_end_timestamp);
    return false;
  }

  {
    Lock lock_imu(imu_mutex_);
    current_imu_data_.clear();
    double imu_time = imu_buffer_.front().timestamp;

    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_timestamp)) {
      imu_time = imu_buffer_.front().timestamp;
      if (imu_time > lidar_end_timestamp) break;
      current_imu_data_.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
  }

  {
    Lock lock_lidar(lidar_mutex_);
    cloud_buffer_.pop_front();
  }
  return true;
}

void FastLIOMapper::ResetMapper() {
  last_imu_timestamp_ = 0;
  imu_buffer_.clear();
  cloud_buffer_.clear();
  current_imu_data_.clear();
  lidar_mean_scan_time_ = 0;
  scan_num_ = 0;
  sensor_disconnected_flag_ = false;

  // 重新初始化Fast-LIO核心
  fast_lio_core_ = std::make_shared<FastLIOCore>();
  fast_lio_core_->SetParams(extrinsic_params_, lio_params_);
  fast_lio_core_->InitializeAll();

  // 重新初始化看门狗
  lidar_watch_dog_->Reset();
  imu_watch_dog_->Reset();

  AINFO_F("[FastLIOMapper] Reset Fast-LIO Mapper");
}

bool FastLIOMapper::CheckSensorTrigger() {
  bool ret = true;
  if (!lidar_watch_dog_->OK()) {
    ret = false;
    AWARN_F("[FastLIOMapper] Lidar trigger timeout");
  }

  if (!imu_watch_dog_->OK()) {
    ret = false;
    AWARN_F("[FastLIOMapper] IMU trigger timeout");
  }

  return ret;
}

bool FastLIOMapper::SensorDisconnected() { return sensor_disconnected_flag_; }

}  // namespace multi_sensor_mapping