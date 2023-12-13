#include "multi_sensor_mapping/param/tiny_location_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

TinyLocationParams::TinyLocationParams(std::string _name)
    : auto_start(false),
      target_resolution(2.0),
      source_resolution(1.0),
      num_cores(1) {
  name_ = _name;
}

void TinyLocationParams::Load(std::string _path_of_yaml) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(_path_of_yaml);
  } catch (...) {
    AERROR_F(
        "The format of config file [ %s ] is wrong, Please check (e.g. "
        "indentation).",
        _path_of_yaml.c_str());
    return;
  }

  utils::YamlRead<bool>(config, "auto_start", auto_start, false);
  utils::YamlRead<double>(config, "target_resolution", target_resolution, 1.0);
  utils::YamlRead<double>(config, "source_resolution", source_resolution, 1.0);
  utils::YamlRead<double>(config, "registration_interval",
                          registration_interval, 5.0);

  utils::YamlRead<int>(config, "num_cores", num_cores, 1);
  utils::YamlRead<float>(config, "direction_offset", direction_offset, 0);
}
void TinyLocationParams::Print() {
  std::cout << "\033[1;32m----> Tiny Location Params <----\033[0m" << std::endl;

  PrintLine("auto_start", auto_start);
  PrintLine("target_resolution", target_resolution);
  PrintLine("source_resolution", source_resolution);
  PrintLine("registration_interval", registration_interval);
  PrintLine("num_cores", num_cores);
  PrintLine("direction_offset", direction_offset);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping