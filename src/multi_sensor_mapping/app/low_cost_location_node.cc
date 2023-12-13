#include <ros/ros.h>

#include <csignal>

#include "config.h"
#include "multi_sensor_mapping/core/low_cost_locator_manager.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "version.h"

using namespace multi_sensor_mapping;

bool finish_flag = false;

void SiginalHandler(int sig) {
  AINFO_F("Low-Cost-Mapping is stopping.....");
  // loguru::shutdown();

  finish_flag = true;
}

int main(int argc, char** argv) {
  std::signal(SIGINT, SiginalHandler);

  ros::init(argc, argv, "low_cost_mapping_node");
  ros::NodeHandle nh_param("~");

  std::string msm_version = std::string(PROJECT_VERSION);
  AINFO_F("Low-Cost-Mapping Project Version  < %s > ", msm_version.c_str());

  std::string config_name;
  std::string map_path;
  std::string record_path;
  bool enable_record_pose;

  nh_param.param<std::string>("config_name", config_name, "");
  nh_param.param<std::string>("map_path", map_path, "");
  nh_param.param<bool>("enable_record_pose", enable_record_pose, false);
  nh_param.param<std::string>("record_path", record_path, "");

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/" + config_name;

  std::cout << param_set_path << std::endl;
  std::cout << map_path << std::endl;

  std::shared_ptr<LowCostLocatorManager> locator_manager(
      new LowCostLocatorManager);
  locator_manager->SetParamSetPath(param_set_path);

  locator_manager->SetMapPath(map_path);

  if (enable_record_pose) {
    locator_manager->SetRecordPath(record_path);
  }

  locator_manager->Init();

  locator_manager->Start();

  // int cnt = 0;
  float current_map_resolution = 1.0;
  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#if ENABLE_2D_VISUALIZATION
    std::cout << "Type in  <c / d> for map resolution" << std::endl;
    char con = std::getchar();
    if (con == 'c') {
      current_map_resolution += 0.1;
      std::cout << "External command : Increase resolution : "
                << current_map_resolution << std::endl;
      locator_manager->SetMapResolution(current_map_resolution);
    } else if (con == 'd') {
      current_map_resolution -= 0.1;
      std::cout << "External command : Reduce resolution : "
                << current_map_resolution << std::endl;
      locator_manager->SetMapResolution(current_map_resolution);
    }

#endif

    ros::spinOnce();
  }

  locator_manager->Stop();

  return 0;
}