#include "multi_sensor_mapping/core/clins_mapper.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/core/clins_core.h"
#include "multi_sensor_mapping/frontend/imu/imu_initializer.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/param/clins_params.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/data_manager.h"
#include "multi_sensor_mapping/utils/rslidar_decoder.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

ClinsMapper::ClinsMapper() : LidarMapperBase() {}

void ClinsMapper::SetParams(const std::shared_ptr<ParamSet>& _param_set) {
  LidarMapperBase::SetParams(_param_set);
  mapper_params_ = _param_set->GetClinsParams();
  mapper_params_->Print();
}

void ClinsMapper::Init() {
  AINFO_F("[ClinsMapper] Init Lidar-IMU Mapper");

  // 检测参数
  if (!mapper_params_ || !sensor_params_ || !extrinsic_params_) {
    AWARN_F("[ClinsMapper] No input Params");
    RunException(MSMError(CODE_NO_INPUT_PARAMS));
    return;
  }

  clins_core_ = std::make_shared<ClinsCore>();
  clins_core_->SetParams(sensor_params_, extrinsic_params_, mapper_params_);

  clins_core_->InitializeAll();

  data_converter_ = std::make_shared<DataConverter>(
      mapper_params_->lidar_min_distance, mapper_params_->lidar_max_distance);
  data_manager_ = std::make_shared<DataManager>(data_converter_);

  init_done_flag_ = true;
}

void ClinsMapper::Start() {
  if (start_flag_) {
    AINFO_F("[ClinsMapper] Mapping process thread have already running");
    return;
  }
  if (!init_done_flag_) {
    AWARN_F("[ClinsMapper] Please init system before start");
    RunException(MSMError(CODE_START_BEFORE_INIT));
    return;
  }

  process_thread_ = std::thread(std::bind(&ClinsMapper::LoopProcess, this));

  exit_process_flag_ = false;
}

void ClinsMapper::Stop() {
  if (!start_flag_) {
    return;
  }

  exit_process_flag_ = true;
  process_thread_.join();

  start_flag_ = false;
  init_done_flag_ = false;
}

bool ClinsMapper::ProcessMapping() {
  boost::filesystem::path bag_path(bag_path_);
  if (bag_path.extension() != ".bag") {
    AWARN_F("[ClinsMapper] Bag path error , bag path : %s ", bag_path_.c_str());
    RunException(MSMError(CODE_NO_INPUT_PARAMS));
    return false;
  }

  AINFO_F("[ClinsMapper] Continuous Time Lidar IMU Mapping Start ");

  // Step1 构建缓存文件夹
  std::string cache_path =
      bag_path.parent_path().string() + "/" + bag_path.stem().string();

  // if (!boost::filesystem::exists(cache_path)) {
  //   boost::filesystem::create_directory(cache_path);
  // }

  // Step2 读取数据
  if (!data_manager_->ReadDataFromRosbag2(bag_path_, sensor_params_)) {
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

  double lidar_num_inv = 1.0 / (double)(data_manager_->GetLidarDataCount());

  // Step5 速腾packets解算
  std::shared_ptr<RslidarDecoder> rslidar_decoder_ptr;
  if (sensor_params_->MajorLidarFormat() >=
      LidarDataFormat::RS_PACKET_RSHELIOS_16P) {
    rslidar_decoder_ptr =
        std::make_shared<RslidarDecoder>(sensor_params_->MajorLidarFormat());
  }

  // Step6 读取数据包，激光里程计
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

  TimedCloudData2 last_cloud_data;
  TimedCloudData2 cloud_data;
  bool cloud_ready_flag = false;

  for (rosbag::MessageInstance const m : view) {
    const std::string& topic = m.getTopic();
    if (sensor_params_->MajorLidarTopic() == topic) {
      std::string data_type = m.getDataType();
      if (data_type == std::string("sensor_msgs/PointCloud2")) {
        /// PointCloud2 数据处理
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        if (sensor_params_->MajorLidarFormat() == LidarDataFormat::RSLIDAR) {
          cloud_data = data_converter_->ConvertRsCloudData2(lidar_msg);
          cloud_ready_flag = true;
        } else if (sensor_params_->MajorLidarFormat() ==
                   LidarDataFormat::RSLIDAR_LEGACY) {
          cloud_data = data_converter_->ConvertLegacyRsCloudData2(lidar_msg);
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
                                           cloud_data.scan_start_timestamp_ns,
                                           cloud_data.scan_end_timestamp_ns);
          AINFO_F("[LidarImuMapper] Current  cloud data %.3f ",
                  cloud_data.scan_start_timestamp_ns * 1e-9);
          cloud_ready_flag = true;
        }
      }
      process_lidar_cnt++;
    }

    if (!cloud_ready_flag) {
      continue;
    }

    cloud_ready_flag = false;
    std::vector<IMUData2> imu_data_vec;
    if (!last_cloud_data.cloud) {
      if (cloud_data.scan_start_timestamp_ns > data_start_time_ns_) {
        last_cloud_data = cloud_data;
      }
    } else {
      if (!data_manager_->GetImuDataInterval(
              imu_data_vec, last_cloud_data.scan_start_timestamp_ns,
              cloud_data.scan_start_timestamp_ns)) {
        AWARN_F(
            "[ClinsMapper] Get IMU wrong , Last cloud time %.3f, current "
            "cloud data %.3f ",
            last_cloud_data.scan_start_timestamp_ns * 1e-9,
            cloud_data.scan_start_timestamp_ns * 1e-9);
      }

      clins_core_->OdometryOneScan(last_cloud_data, imu_data_vec);

      std::cout << "Odometry one scan " << std::endl;

      last_cloud_data = cloud_data;

      // // 上传当前姿态
      // RunPutLidarPose(lidar_imu_mapping_core_->GetCurrentPose());

      // // 上传当前帧信息
      // RunPutFrame(lidar_imu_mapping_core_->GetCurrentCloud(),
      //             lidar_imu_mapping_core_->GetCurrentPose());

      // // 上传关键帧姿态
      // if (lidar_imu_mapping_core_->IsKeyFrame()) {
      //   LidarMapUnit::Ptr map_unit = active_session->GetLatestMapUnit();
      //   if (map_unit.get()) {
      //     PoseData pose_data;
      //     pose_data.timestamp = map_unit->timestamp_;
      //     pose_data.position = map_unit->position_;
      //     pose_data.orientation = map_unit->orientation_;
      //     RunPutKeyFrame(map_unit->full_cloud_, pose_data);
      //   }
      // }

      // double mapping_progress = double(process_lidar_cnt) * lidar_num_inv;
      // RunPutMappingProgress(mapping_progress);
      // 上传建图进度
    }

    std::getchar();

    if (exit_process_flag_) {
      break;
    }
  }

  return true;
}

void ClinsMapper::LoopProcess() {
  ProcessMapping();

  std::cout << "LoopProcess" << std::endl;

  while (!exit_process_flag_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool ClinsMapper::StaticInitializationProcess() {
  ImuInitializer imu_initializer(mapper_params_->gravity_norm,
                                 mapper_params_->initial_window_length,
                                 mapper_params_->imu_excite_threshold);
  double start_time;
  int imu_count = 0;
  while (true) {
    IMUData2 imu_data;
    if (data_manager_->GetImuData(imu_data, imu_count)) {
      imu_initializer.FeedImuData(imu_data);
      imu_count++;
    } else {
      break;
    }

    // IMU初始化检测
    IMUState init_state;
    if (imu_count % 50 == 0) {
      if (imu_initializer.CheckImuStaticState(init_state)) {
        // IMU初始化成功
        IMUData2 latest_imu_data = imu_initializer.GetLatestImuData();
        data_start_time_ns_ = latest_imu_data.timestamp_ns;
        // 设置初始状态
        clins_core_->SetInitialState(init_state, latest_imu_data);
        return true;
      }
    }
  }

  return false;
}

bool ClinsMapper::ImuInitializationProcess() {
  IMUState init_state;
  IMUData2 latest_imu_data;
  // IMU激光对齐
  data_manager_->LidarImuData2Alignment(init_state, latest_imu_data);

  // 设置初始状态
  clins_core_->SetInitialState(init_state, latest_imu_data);
  data_start_time_ns_ = latest_imu_data.timestamp_ns;
  return true;
}

}  // namespace multi_sensor_mapping