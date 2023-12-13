#include "multi_sensor_mapping/param/patchwork_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

void PatchWorkParams::Load(std::string _path_of_yaml) {
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

  utils::YamlRead<double>(config, "sensor_height", sensor_height, 1.8);
  utils::YamlRead<int>(config, "num_iter", num_iter, 3);
  utils::YamlRead<int>(config, "num_lpr", num_lpr, 20);
  utils::YamlRead<int>(config, "num_min_pts", num_min_pts, 10);
  utils::YamlRead<double>(config, "th_seeds", th_seeds, 0.3);
  utils::YamlRead<double>(config, "th_dist", th_dist, 0.125);
  utils::YamlRead<double>(config, "th_seeds_v", th_seeds_v, 0.25);
  utils::YamlRead<double>(config, "th_dist_v", th_dist_v, 0.1);
  utils::YamlRead<double>(config, "max_r", max_r, 30);
  utils::YamlRead<double>(config, "min_r", min_r, 0.5);
  utils::YamlRead<double>(config, "uprightness_thr", uprightness_thr, 0.907);
  utils::YamlRead<bool>(config, "enable_RNR", enable_RNR, true);
  utils::YamlRead<bool>(config, "enable_RVPF", enable_RVPF, true);
  utils::YamlRead<bool>(config, "enable_TGR", enable_TGR, true);
  utils::YamlRead<double>(config, "RNR_ver_angle_thr", RNR_ver_angle_thr,
                          -15.0);
  utils::YamlRead<double>(config, "RNR_intensity_thr", RNR_intensity_thr, 100);
  utils::YamlRead<double>(config, "adaptive_seed_selection_margin",
                          adaptive_seed_selection_margin, -1.2);
}

void PatchWorkParams::Print() {
  std::cout << "\033[1;32m----> Patchwork Params <----\033[0m" << std::endl;

  PrintLine("sensor_height", sensor_height);
  PrintLine("num_iter", num_iter);
  PrintLine("num_lpr", num_lpr);
  PrintLine("num_min_pts", num_min_pts);
  PrintLine("th_seeds", th_seeds);
  PrintLine("th_dist", th_dist);
  PrintLine("th_seeds_v", th_seeds_v);
  PrintLine("th_dist_v", th_dist_v);
  PrintLine("max_r", max_r);
  PrintLine("min_r", min_r);
  PrintLine("uprightness_thr", uprightness_thr);
  PrintLine("enable_RNR", enable_RNR);
  PrintLine("enable_RVPF", enable_RVPF);
  PrintLine("enable_TGR", enable_TGR);
  PrintLine("RNR_ver_angle_thr", RNR_ver_angle_thr);
  PrintLine("RNR_intensity_thr", RNR_intensity_thr);
  PrintLine("adaptive_seed_selection_margin", adaptive_seed_selection_margin);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping