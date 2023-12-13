#include <ros/ros.h>

#include <csignal>

#include "config.h"
#include "multi_sensor_mapping/core/odometry_manager.h"
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

  ros::init(argc, argv, "lidar_imu_odometry_node");
  ros::NodeHandle nh_param("~");

  std::string msm_version = std::string(PROJECT_VERSION);
  AINFO_F("Multi-Sensor-Mapping Project Version  < %s > ", msm_version.c_str());

  std::string config_name;

  nh_param.param<std::string>("config_name", config_name, "");

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/" + config_name;

  std::shared_ptr<OdometryManager> odometry_manager(new OdometryManager);
  odometry_manager->SetParamSetPath(param_set_path);
  odometry_manager->Init();

  odometry_manager->Start();

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (finish_flag) {
      odometry_manager->Stop();
      break;
    }
  }
  return 0;
}