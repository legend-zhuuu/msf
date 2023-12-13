#include <ros/ros.h>

#include <csignal>

#include "config.h"
#include "multi_sensor_mapping/core/low_cost_mapper_manager.h"
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
  std::string map_cache_path;

  nh_param.param<std::string>("config_name", config_name, "");
  nh_param.param<std::string>("map_cache_path", map_cache_path, "");

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/" + config_name;
  // std::string pose_icon_path = source_dir + "/docs/_static/pose_icon.jpg";

  std::shared_ptr<LowCostMapperManager> mapper_manager(
      new LowCostMapperManager);
  mapper_manager->SetParamSetPath(param_set_path);
  mapper_manager->Init();
  mapper_manager->SetCachePath(map_cache_path);
  // mapper_manager->SetPoseIconPath(pose_icon_path);
  mapper_manager->Start();

  // int cnt = 0;
  float current_map_resolution = 0.2;
  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "Type in < a > for stop mapping, < b > for stop, <c / d> for "
                 "map resolution"
              << std::endl;

    char con = std::getchar();
    if (con == 'a') {
      std::cout << "External command : Stop Mapping" << std::endl;
      mapper_manager->StopMapping();
    } else if (con == 'b') {
      std::cout << "External command : Stop" << std::endl;
      break;
    } else if (con == 'c') {
      current_map_resolution += 0.1;
      std::cout << "External command : Increase resolution : "
                << current_map_resolution << std::endl;
      mapper_manager->SetMapResolution(current_map_resolution);
    } else if (con == 'd') {
      current_map_resolution -= 0.1;
      std::cout << "External command : Reduce resolution : "
                << current_map_resolution << std::endl;
      mapper_manager->SetMapResolution(current_map_resolution);
    }
  }

  mapper_manager->Stop();

  return 0;
}