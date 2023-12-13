#include "multi_sensor_mapping/backend/pose_graph_information.h"

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

PoseGraphDatabase::PoseGraphDatabase()
    : loop_closure_constraint_id_(0), gnss_constraint_id_(0) {}

void PoseGraphDatabase::AddLoopClosureConstraint(
    LoopClosureConstraint &_loop_closure_constraint) {
  loop_closure_info_id_vec_.push_back(loop_closure_constraint_id_);
  loop_closure_constraint_info_.insert(
      std::make_pair(loop_closure_constraint_id_, _loop_closure_constraint));
  loop_closure_constraint_id_++;
}

void PoseGraphDatabase::AddGnssConstraint(GNSSConstraint &_gnss_constraint) {
  gnss_constraint_id_vec_.push_back(gnss_constraint_id_);
  gnss_constraint_info_.insert(
      std::make_pair(gnss_constraint_id_, _gnss_constraint));
  gnss_constraint_id_++;
}

bool PoseGraphDatabase::Save(std::string _cache_path) {
  YAML::Node pose_graph_node;
  pose_graph_node["loop_closure_constraint_num"] =
      int(loop_closure_constraint_info_.size());
  pose_graph_node["gnss_constraint_num"] = int(gnss_constraint_info_.size());

  for (size_t i = 0; i < loop_closure_info_id_vec_.size(); i++) {
    int loop_closure_id = loop_closure_info_id_vec_[i];
    LoopClosureConstraint lcc = loop_closure_constraint_info_[loop_closure_id];

    YAML::Node lc_node;
    lc_node["loop_closure_id"] = loop_closure_id;
    lc_node["front_id"] = lcc.front_id;
    lc_node["back_id"] = lcc.back_id;
    lc_node["valid_flag"] = lcc.valid_flag;
    lc_node["score"] = lcc.reg_score;
    lc_node["delta_position"] = utils::ToVector(lcc.delta_position);
    lc_node["delta_rotation"] = utils::ToVector(lcc.delta_rotation);
    pose_graph_node["loop_closure_info"].push_back(lc_node);
  }

  for (size_t i = 0; i < gnss_constraint_id_vec_.size(); i++) {
    int gnss_info_id = gnss_constraint_id_vec_[i];
    GNSSConstraint gc = gnss_constraint_info_[gnss_info_id];

    YAML::Node gnss_node;
    gnss_node["gnss_id"] = gnss_info_id;
    gnss_node["id"] = gc.id;
    gnss_node["use_gps_elevation"] = gc.use_gps_elevation;
    gnss_node["valid_flag"] = gc.valid_flag;
    gnss_node["gnss_pose"] = utils::ToVector(gc.gnss_pose);
    gnss_node["gnss_noise"] = utils::ToVector(gc.gnss_noise);
    pose_graph_node["gnss_constraint_info"].push_back(gnss_node);
  }

  // 保存yaml
  std::string pg_db_path = _cache_path + "/pose_graph_database.yaml";
  std::ofstream pose_graph_info_file(pg_db_path.c_str());
  pose_graph_info_file << pose_graph_node;
  pose_graph_info_file.close();

  AINFO_F("[PoseGraphDatabase] Save pose graph success");
  return true;
}

bool PoseGraphDatabase::Load(std::string _cache_path) {
  if (!boost::filesystem::exists(_cache_path)) {
    AWARN_F("[PoseGraphDatabase] Folder does not exist");
    return false;
  }

  std::string pg_db_path = _cache_path + "/pose_graph_database.yaml";
  YAML::Node pose_graph_node;
  try {
    pose_graph_node = YAML::LoadFile(pg_db_path);
  } catch (...) {
    AWARN_F(
        " [PoseGraphDatabase] The format of config file [ %s ] is wrong, "
        "Please "
        "check (e.g. indentation).",
        pg_db_path.c_str());
    return false;
  }

  int loop_closure_constraint_num, gnss_constraint_num;
  utils::YamlRead<int>(pose_graph_node, "loop_closure_constraint_num",
                       loop_closure_constraint_num, 0);
  utils::YamlRead<int>(pose_graph_node, "gnss_constraint_num",
                       gnss_constraint_num, 0);

  if (utils::CheckYamlSubNode(pose_graph_node, "loop_closure_info")) {
    YAML::Node loop_closure_node =
        utils::YamlSubNodeAbort(pose_graph_node, "loop_closure_info");
    for (size_t i = 0; i < loop_closure_node.size(); i++) {
      int lcc_id = loop_closure_node[i]["loop_closure_id"].as<int>();
      LoopClosureConstraint lcc;
      lcc.front_id = loop_closure_node[i]["front_id"].as<int>();
      lcc.back_id = loop_closure_node[i]["back_id"].as<int>();
      lcc.valid_flag = loop_closure_node[i]["valid_flag"].as<bool>();
      lcc.reg_score = loop_closure_node[i]["score"].as<double>();

      std::vector<double> position_vec =
          loop_closure_node[i]["delta_position"].as<std::vector<double>>();
      std::vector<double> quater_coeff_vec =
          loop_closure_node[i]["delta_rotation"].as<std::vector<double>>();

      lcc.delta_position =
          Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
      lcc.delta_rotation =
          Eigen::Quaterniond(quater_coeff_vec[3], quater_coeff_vec[0],
                             quater_coeff_vec[1], quater_coeff_vec[2]);
      loop_closure_info_id_vec_.push_back(lcc_id);
      loop_closure_constraint_info_.insert(std::make_pair(lcc_id, lcc));
    }

    if (!loop_closure_info_id_vec_.empty()) {
      loop_closure_constraint_id_ = loop_closure_info_id_vec_.back() + 1;
    }
  }

  if (utils::CheckYamlSubNode(pose_graph_node, "gnss_constraint_info")) {
    YAML::Node gnss_constraint_node =
        utils::YamlSubNodeAbort(pose_graph_node, "gnss_constraint_info");
    // 加载GNSS约束信息

    for (size_t i = 0; i < gnss_constraint_node.size(); i++) {
      int gc_id = gnss_constraint_node[i]["gnss_id"].as<int>();
      GNSSConstraint gc;
      gc.id = gnss_constraint_node[i]["id"].as<int>();
      gc.use_gps_elevation =
          gnss_constraint_node[i]["use_gps_elevation"].as<bool>();
      gc.valid_flag = gnss_constraint_node[i]["valid_flag"].as<bool>();

      std::vector<double> pos_vec =
          gnss_constraint_node[i]["gnss_pose"].as<std::vector<double>>();
      std::vector<double> noise_vec =
          gnss_constraint_node[i]["gnss_noise"].as<std::vector<double>>();

      gc.gnss_pose = Eigen::Vector3d(pos_vec[0], pos_vec[1], pos_vec[2]);
      gc.gnss_noise = Eigen::Vector3d(noise_vec[0], noise_vec[1], noise_vec[2]);

      gnss_constraint_id_vec_.push_back(gc_id);
      gnss_constraint_info_.insert(std::make_pair(gc_id, gc));
    }
    if (!gnss_constraint_id_vec_.empty()) {
      gnss_constraint_id_ = gnss_constraint_id_vec_.back() + 1;
    }
  }

  if (loop_closure_constraint_num != loop_closure_constraint_info_.size() ||
      gnss_constraint_num != gnss_constraint_info_.size()) {
    AINFO_F("[PoseGraphDatabase] Load pose graph database failed!");
    return false;
  }

  return true;
}

void PoseGraphDatabase::ClearLoopClosureConstraint() {
  loop_closure_info_id_vec_.clear();
  loop_closure_constraint_info_.clear();
  loop_closure_constraint_id_ = 0;
}

void PoseGraphDatabase::ClearGnssCOnstraint() {
  gnss_constraint_id_vec_.clear();
  gnss_constraint_info_.clear();
  gnss_constraint_id_ = 0;
}

}  // namespace multi_sensor_mapping
