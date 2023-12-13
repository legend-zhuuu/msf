#include "multi_sensor_mapping/map/lidar_map_session.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  LidarMapSession session;

  std::string session_info_file = "/home/hkw/session";

  session.Save(session_info_file);

  return 0;
}