#include "multi_sensor_mapping/core/lidar_imu_mapper.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/core/fast_lio_core.h"
#include "multi_sensor_mapping/core/lidar_imu_mapping_core.h"
#include "multi_sensor_mapping/frontend/imu/imu_initializer.h"
#include "multi_sensor_mapping/frontend/imu/imu_pose_predictor.h"
#include "multi_sensor_mapping/frontend/lidar/feature_extractor.h"
#include "multi_sensor_mapping/frontend/lidar/loam_scan_matcher.h"
#include "multi_sensor_mapping/frontend/lidar/scan_undistortion.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/data_manager.h"
#include "multi_sensor_mapping/utils/rslidar_decoder.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

/// @brief 是否启用FAST-LIO
bool enable_fast_lio_flag = false;
/// @brief 激光-IMU的旋转外参
Eigen::Quaterniond lidar_to_imu_rotation;
/// @brief IMU-激光雷达的旋转外参
Eigen::Quaterniond imu_to_lidar_rotation;

LidarImuMapper::LidarImuMapper() : LidarMapperBase() {}

void LidarImuMapper::SetParams(const std::shared_ptr<ParamSet> &_param_set) {
  LidarMapperBase::SetParams(_param_set);
  mapper_params_ = _param_set->GetLidarImuMappingParams();
  lio_params_ = _param_set->GetFastLIOParams();
  mapper_params_->Print();
}

void LidarImuMapper::Init() {
  AINFO_F("[LidarImuMapper] Init Lidar-IMU Mapper");

  // 检测参数
  if (!mapper_params_ || !sensor_params_ || !extrinsic_params_) {
    AWARN_F("[LidarImuMapper] No input Params");
    RunException(MSMError(CODE_NO_INPUT_PARAMS));
    return;
  }

  lidar_imu_mapping_core_ = std::make_shared<LidarImuMappingCore>();
  lidar_imu_mapping_core_->SetParams(sensor_params_, extrinsic_params_,
                                     mapper_params_);
  lidar_imu_mapping_core_->InitializeAll();

  if (mapper_params_->match_method == MatchMethod::MATCH_METHOD_FAST_LIO) {
    // 初始化Fast-LIO核心
    fast_lio_core_ = std::make_shared<FastLIOCore>();
    fast_lio_core_->SetParams(extrinsic_params_, lio_params_);
    fast_lio_core_->InitializeAll();

    enable_fast_lio_flag = true;
  }

  data_converter_ = std::make_shared<DataConverter>(
      mapper_params_->lidar_min_distance, mapper_params_->lidar_max_distance);
  data_manager_ = std::make_shared<DataManager>(data_converter_);

  /// 计算激光-IMU之间的相对外参
  Eigen::Quaterniond major_lidar_quater = Eigen::Quaterniond(
      extrinsic_params_->lidar_to_baselink[sensor_params_->major_lidar_]
          .block<3, 3>(0, 0));
  lidar_to_imu_rotation =
      extrinsic_params_->imu_to_baselink_rotation_.inverse() *
      major_lidar_quater;
  imu_to_lidar_rotation = lidar_to_imu_rotation.inverse();

  init_done_flag_ = true;
}

void LidarImuMapper::Start() {
  if (start_flag_) {
    AINFO_F("[LidarImuMapper] Mapping process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[LidarImuMapper] Please init system before start");
    RunException(MSMError(CODE_START_BEFORE_INIT));
    return;
  }

  exit_process_flag_ = false;
  // 主处理线程
  process_thread_ = std::thread(std::bind(&LidarImuMapper::LoopProcess, this));

  start_flag_ = true;
}

void LidarImuMapper::Stop() {
  if (!start_flag_) {
    return;
  }

  exit_process_flag_ = true;
  process_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;
}

bool LidarImuMapper::ProcessMapping() {
  boost::filesystem::path bag_path(bag_path_);
  if (bag_path.extension() != ".bag") {
    AWARN_F("[LidarImuMapper] Bag path error , bag path : %s ",
            bag_path_.c_str());
    RunException(MSMError(CODE_NO_INPUT_PARAMS));
    return false;
  }

  AINFO_F("[LidarImuMapper] Lidar IMU Mapping Start ");

  // Step1 构建缓存文件夹
  std::string cache_path =
      bag_path.parent_path().string() + "/" + bag_path.stem().string();

  // if (!boost::filesystem::exists(cache_path)) {
  //   boost::filesystem::create_directory(cache_path);
  // }

  // Step2 读取数据
  if (!data_manager_->ReadDataFromRosbag(bag_path_, sensor_params_)) {
    return false;
  }

  // Step3 系统初始化，重力对齐
  if (StaticInitializationProcess()) {
    AINFO_F("[LidarImuMapper] System static init success!");
  } else {
    // 初始化失败，则直接使用IMU的方向数据
    ImuInitializationProcess();
  }

  // Step4 创建新的地图任务
  LidarMapSession::Ptr active_session =
      LidarMapSession::Ptr(new LidarMapSession(0, bag_path.stem().string()));
  active_session->SetCachePath(cache_path);
  lidar_imu_mapping_core_->SetLidarMapSession(active_session);

  // Step5 激光里程计
  double lidar_num_inv = 1.0 / (double)(data_manager_->GetLidarDataCount());

  // Step6 速腾packets解算
  std::shared_ptr<RslidarDecoder> rslidar_decoder_ptr;
  if (sensor_params_->MajorLidarFormat() >=
      LidarDataFormat::RS_PACKET_RSHELIOS_16P) {
    rslidar_decoder_ptr =
        std::make_shared<RslidarDecoder>(sensor_params_->MajorLidarFormat());
  }

  // Step7 读取数据包，激光里程计
  rosbag::Bag bag;
  bag.open(bag_path_, rosbag::bagmode::Read);

  rosbag::View view;
  std::vector<std::string> topics;
  // TODO 当前版本只支持单激光建图
  topics.push_back(sensor_params_->MajorLidarTopic());

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start_time_);
  ros::Time time_finish = (bag_durr_ < 0)
                              ? view_full.getEndTime()
                              : time_init + ros::Duration(bag_durr_);
  ros::Duration delta_durr = ros::Duration(0.1);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init - delta_durr,
                time_finish + delta_durr);

  size_t process_lidar_cnt = 0;

  TimedCloudData last_cloud_data;
  TimedCloudData cloud_data;
  bool cloud_ready_flag = false;
  for (rosbag::MessageInstance const m : view) {
    const std::string &topic = m.getTopic();
    if (sensor_params_->MajorLidarTopic() == topic) {
      std::string data_type = m.getDataType();
      if (data_type == std::string("sensor_msgs/PointCloud2")) {
        /// PointCloud2 数据处理
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        if (sensor_params_->MajorLidarFormat() == LidarDataFormat::RSLIDAR) {
          cloud_data = data_converter_->ConvertRsCloudData(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params_->MajorLidarFormat() ==
                   LidarDataFormat::RSLIDAR_LEGACY) {
          cloud_data = data_converter_->ConvertLegacyRsCloudData(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params_->MajorLidarFormat() ==
                   LidarDataFormat::VELODYNE) {
          cloud_data = data_converter_->ConvertVelCloudData(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params_->MajorLidarFormat() ==
                   LidarDataFormat::HESAI) {
          cloud_data = data_converter_->ConvertHesaiCloudData(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params_->MajorLidarFormat() ==
                   LidarDataFormat::LIVOX) {
          cloud_data = data_converter_->ConvertLivoxCloudData(lidar_msg);
          cloud_ready_flag = true;
        }

      } else if (data_type == "rslidar_msg/RslidarPacket") {
        /// RslidarPacket 数据处理
        rslidar_msg::RslidarPacket::ConstPtr packet_msg =
            m.instantiate<rslidar_msg::RslidarPacket>();

        if (rslidar_decoder_ptr->DecodePacket(*packet_msg)) {
          std::shared_ptr<RsPointCloudMsg> cloud =
              rslidar_decoder_ptr->GetCloud();
          RsRTPointCloudPtr raw_cloud(new RsRTPointCloud);
          *raw_cloud = *cloud;

          data_converter_->PreprocessCloud(raw_cloud, cloud_data.cloud,
                                           cloud_data.timestamp);
          // AINFO_F("[LidarImuMapper] Current  cloud data %.3f ",
          //         cloud_data.timestamp);
          cloud_ready_flag = true;
        }
      }
      process_lidar_cnt++;
    }

    if (!cloud_ready_flag) {
      continue;
    }

    cloud_ready_flag = false;
    std::vector<IMUData> imu_data_vec;
    if (!last_cloud_data.cloud) {
      last_cloud_data = cloud_data;
    } else {
      if (!data_manager_->GetImuDataInterval(
              imu_data_vec, last_cloud_data.timestamp, cloud_data.timestamp)) {
        AWARN_F(
            "[LidarImuMapper] Get IMU wrong , Last cloud time %.3f, current "
            "cloud data %.3f ",
            last_cloud_data.timestamp, cloud_data.timestamp);
      }

      if (enable_fast_lio_flag) {
        std::deque<IMUData> imu_deque;
        for (size_t i = 0; i < imu_data_vec.size(); i++) {
          imu_deque.push_back(imu_data_vec[i]);
        }
        fast_lio_core_->OdometryOnceScan(last_cloud_data, imu_deque);
        last_cloud_data = cloud_data;

        PoseData pose_data = fast_lio_core_->GetCurrentPose();
        CloudTypePtr cloud_data = fast_lio_core_->GetCurrentCloud();
        // 上传当前姿态
        RunPutLidarPose(pose_data);

        // 上传当前帧信息
        RunPutFrame(cloud_data, pose_data);

        // 上传关键帧姿态
        if (cloud_data->size() > 100) {
          if (active_session->GetMapUnitSize() == 0 ||
              CheckKeyScan(pose_data)) {
            CloudTypePtr key_frame_cloud(new CloudType);
            utils::UniformSampleCloud(*cloud_data, *key_frame_cloud, 0.1);

            Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
            pose_mat.block<3, 3>(0, 0) =
                pose_data.orientation.toRotationMatrix();
            pose_mat.block<3, 1>(0, 3) = pose_data.position;
            std::shared_ptr<LidarMapUnit> map_unit(new LidarMapUnit(
                pose_data.timestamp, key_frame_cloud, pose_mat));
            active_session->AddMapUnit(map_unit);

            RunPutKeyFrame(cloud_data, pose_data);
          }
        }
      } else {
        for (size_t i = 0; i < imu_data_vec.size(); i++) {
          ConvertImuToLidarFrame(imu_data_vec[i]);
        }

        lidar_imu_mapping_core_->OdometryOneScan(last_cloud_data, imu_data_vec);

        last_cloud_data = cloud_data;

        // 上传当前姿态
        RunPutLidarPose(lidar_imu_mapping_core_->GetCurrentPose());

        // 上传当前帧信息
        RunPutFrame(lidar_imu_mapping_core_->GetCurrentCloud(),
                    lidar_imu_mapping_core_->GetCurrentPose());

        // 上传关键帧姿态
        if (lidar_imu_mapping_core_->IsKeyFrame()) {
          LidarMapUnit::Ptr map_unit = active_session->GetLatestMapUnit();
          if (map_unit.get()) {
            PoseData pose_data;
            pose_data.timestamp = map_unit->timestamp_;
            pose_data.position = map_unit->position_;
            pose_data.orientation = map_unit->orientation_;
            RunPutKeyFrame(map_unit->full_cloud_, pose_data);
          }
        }
      }

      double mapping_progress = double(process_lidar_cnt) * lidar_num_inv;
      RunPutMappingProgress(mapping_progress);
      // 上传建图进度
    }

    // usleep(50000);

    if (exit_process_flag_) {
      break;
    }
  }
  RunPutMappingProgress(1.0);

  // Step8 保存session
  // active_session->Save(cache_path);
  // 上传session
  if (cb_put_session_) {
    cb_put_session_(active_session);
  }

  return true;
}

bool LidarImuMapper::StaticInitializationProcess() {
  ImuInitializer imu_initializer(mapper_params_->gravity_norm,
                                 mapper_params_->initial_window_length,
                                 mapper_params_->imu_excite_threshold);
  double start_time;
  int imu_count = 0;
  while (true) {
    IMUData imu_data;
    if (data_manager_->GetImuData(imu_data, imu_count)) {
      imu_initializer.FeedImuData(imu_data);
      imu_count++;
    } else {
      break;
    }

    // IMU初始化检测
    if (imu_count % 50 == 0) {
      if (imu_initializer.CheckImuStaticState(start_time)) {
        // IMU初始化成功
        Eigen::Quaterniond imu_start_rotation = imu_initializer.GetI0ToG();

        Eigen::Quaterniond init_rotation =
            imu_start_rotation * lidar_to_imu_rotation;
        // 设置初始状态
        lidar_imu_mapping_core_->SetInitializationStatus(start_time,
                                                         init_rotation);
        return true;
      }
    }
  }

  return false;
}

bool LidarImuMapper::ImuInitializationProcess() {
  double start_time;
  Eigen::Quaterniond start_imu_orientation;

  data_manager_->LidarImuDataAlignment(start_time, start_imu_orientation);

  Eigen::Quaterniond init_rotation =
      start_imu_orientation * lidar_to_imu_rotation;
  // 设置初始状态
  lidar_imu_mapping_core_->SetInitializationStatus(start_time, init_rotation);

  return true;
}

void LidarImuMapper::LoopProcess() {
  ProcessMapping();

  while (!exit_process_flag_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void LidarImuMapper::ConvertImuToLidarFrame(IMUData &imu_data) {
  imu_data.gyro = imu_to_lidar_rotation * imu_data.gyro;
  imu_data.accel = imu_to_lidar_rotation * imu_data.accel;
  imu_data.orientation = imu_data.orientation * lidar_to_imu_rotation;
}

bool LidarImuMapper::CheckKeyScan(const PoseData &_pose) {
  static Eigen::Vector3d last_key_frame_pose(0, 0, 0);

  double delta_pose_sqr = (_pose.position(0) - last_key_frame_pose(0)) *
                              (_pose.position(0) - last_key_frame_pose(0)) +
                          (_pose.position(1) - last_key_frame_pose(1)) *
                              (_pose.position(1) - last_key_frame_pose(1));

  if (delta_pose_sqr > mapper_params_->keyframe_adding_distance_threshold *
                           mapper_params_->keyframe_adding_distance_threshold) {
    last_key_frame_pose = _pose.position;
    return true;
  }
  return false;
}

}  // namespace multi_sensor_mapping
