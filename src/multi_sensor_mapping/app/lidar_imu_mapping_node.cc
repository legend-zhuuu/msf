#include <ros/ros.h>

#include <csignal>

#include "config.h"
#include "multi_sensor_mapping/core/mapper_manager.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "version.h"

using namespace multi_sensor_mapping;

bool finish_flag = false;

static void SiginalHandler(int sig) {
  AINFO_F("Multi-Sensor-Mapping is stopping.....");
  // loguru::shutdown();

  finish_flag = true;
}

int main(int argc, char** argv) {
  std::signal(SIGINT, SiginalHandler);

  ros::init(argc, argv, "lidar_imu_mapping_node");
  ros::NodeHandle nh_param("~");

  std::string msm_version = std::string(PROJECT_VERSION);
  AINFO_F("Multi-Sensor-Mapping Project Version  < %s > ", msm_version.c_str());

  std::string bag_path;
  std::string config_name;
  double start_time, mapping_durring;

  nh_param.param<std::string>("bag_path", bag_path, "");
  nh_param.param<std::string>("config_name", config_name, "");
  nh_param.param<double>("start_time", start_time, 0);
  nh_param.param<double>("mapping_durring", mapping_durring, -1);

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/" + config_name;

  MapperManager::Ptr mapper_manager(new MapperManager);
  mapper_manager->SetParamSetPath(param_set_path);
  mapper_manager->SetBagPath(bag_path, start_time, mapping_durring);
  mapper_manager->Init();

  mapper_manager->Start();

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (finish_flag) {
      mapper_manager->Stop();
      break;
    }
  }
  return 0;
}