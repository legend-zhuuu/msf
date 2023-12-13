#include "multi_sensor_mapping/map/lidar_map_session.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <unordered_set>

#include "multi_sensor_mapping/backend/loop_closure_detector.h"
#include "multi_sensor_mapping/backend/pose_graph_information.h"
#include "multi_sensor_mapping/map/grid_map_generator.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"
#include "multi_sensor_mapping/utils/utils_log.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

LidarMapSession::LidarMapSession()
    : is_base_session_(true),
      session_version_("2.0"),
      session_index_(0),
      session_name_(" "),
      update_flag_(false),
      map_units_translation_cloud_(new CloudType),
      gnss_alignment_flag_(false),
      gnss_origin_coordinate_(Eigen::Vector3d(0, 0, 0)) {
  surrounding_key_poses_filter_.setLeafSize(2.0, 2.0, 2.0);

  pose_graph_database_ptr_ = PoseGraphDatabase::Ptr(new PoseGraphDatabase);
}

LidarMapSession::LidarMapSession(int _session_id, std::string _session_name)
    : is_base_session_(true),
      session_version_("2.0"),
      session_index_(_session_id),
      session_name_(_session_name),
      update_flag_(false),
      map_units_translation_cloud_(new CloudType),
      gnss_alignment_flag_(false),
      gnss_origin_coordinate_(Eigen::Vector3d(0, 0, 0)) {
  surrounding_key_poses_filter_.setLeafSize(2.0, 2.0, 2.0);

  pose_graph_database_ptr_ = PoseGraphDatabase::Ptr(new PoseGraphDatabase);
}

void LidarMapSession::SetCachePath(std::string _path) { cache_path_ = _path; }

void LidarMapSession::AddMapUnit(LidarMapUnit::Ptr map_unit) {
  map_units_.push_back(map_unit);

  PointType p;
  p.x = map_unit->position_(0);
  p.y = map_unit->position_(1);
  p.z = map_unit->position_(2);
  p.intensity = map_units_translation_cloud_->size();

  // 保存距离
  if (key_scan_distance_.empty()) {
    key_scan_distance_.push_back(0);
  } else {
    double distance =
        key_scan_distance_.back() +
        utils::PointDistance(p, map_units_translation_cloud_->points.back());
    key_scan_distance_.push_back(distance);
  }
  map_units_translation_cloud_->push_back(p);

  update_flag_ = true;
}

int LidarMapSession::ExtractSurroundingKeyFrames(
    double cur_timestamp, double key_frame_search_radius,
    CloudTypePtr& surrounding_corner_cloud,
    CloudTypePtr& surrounding_surface_cloud) {
  if (Empty()) return 0;
  if (!update_flag_) return -1;

  CloudTypePtr surrounding_key_poses(new CloudType);

  // std::vector<int> point_search_ind;
  // std::vector<float> point_search_sq_dis;

  // pcl::KdTreeFLANN<PointType> kdtree_surround_pose;
  // kdtree_surround_pose.setInputCloud(map_units_translation_cloud_);
  // kdtree_surround_pose.radiusSearch(map_units_translation_cloud_->back(),
  //                                   key_frame_search_radius,
  //                                   point_search_ind, point_search_sq_dis);

  // for (size_t i = 0; i < point_search_ind.size(); i++) {
  //   int id = point_search_ind[i];
  //   surrounding_key_poses->push_back(map_units_translation_cloud_->points[id]);
  // }

  /// 添加最近的20帧关键帧
  int key_frame_count = 0;
  for (int i = map_units_.size() - 1; i >= 0; i--) {
    surrounding_key_poses->push_back(map_units_translation_cloud_->points[i]);
    key_frame_count++;
    if (key_frame_count > 40) break;
  }

  /// step3 提取corner和surface点云
  surrounding_corner_cloud->clear();
  surrounding_surface_cloud->clear();
  for (size_t i = 0; i < surrounding_key_poses->size(); i++) {
    int index = (int)(surrounding_key_poses->points[i].intensity);

    CloudType corner_global, surface_global;
    pcl::transformPointCloud(*(map_units_[index]->corner_feature_cloud_ds_),
                             corner_global, map_units_[index]->position_,
                             map_units_[index]->orientation_);
    pcl::transformPointCloud(*(map_units_[index]->surface_feature_cloud_ds_),
                             surface_global, map_units_[index]->position_,
                             map_units_[index]->orientation_);
    *surrounding_corner_cloud += corner_global;
    *surrounding_surface_cloud += surface_global;
  }
  update_flag_ = false;
  return (int)surrounding_key_poses->size();
}

CloudTypePtr LidarMapSession::ExtractTopViewCloud(
    double _delta_slice_radius, double _delta_slice_height,
    double _delta_slice_thickness) {
  CloudTypePtr top_view_cloud(new CloudType);

  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];
    CloudType cloud_gravity_aligment;
    pcl::transformPointCloud(*(unit->full_cloud_), cloud_gravity_aligment,
                             Eigen::Vector3d(0, 0, 0), unit->orientation_);
    double extract_dis = unit->slice_attribute_(0) + _delta_slice_radius;
    double min_height = unit->slice_attribute_(1) + _delta_slice_height;
    double max_height =
        min_height + unit->slice_attribute_(2) + _delta_slice_thickness;
    CloudType cloud_slice;
    for (size_t i = 0; i < cloud_gravity_aligment.size(); i++) {
      PointType p = cloud_gravity_aligment.points[i];
      if (utils::PointRadius(p) > extract_dis) continue;
      if (p.z > max_height || p.z < min_height) continue;
      // 直接转换到全局
      p.x += unit->position_(0);
      p.y += unit->position_(1);
      p.z += unit->position_(2);
      cloud_slice.push_back(p);
    }

    *top_view_cloud += cloud_slice;
  }

  return top_view_cloud;
}

CloudTypePtr LidarMapSession::ExtractTopViewCloud2(double _slice_radius,
                                                   double _slice_height,
                                                   double _slice_thickness) {
  CloudTypePtr top_view_cloud(new CloudType);

  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];
    CloudType cloud_gravity_aligment;
    pcl::transformPointCloud(*(unit->full_cloud_), cloud_gravity_aligment,
                             Eigen::Vector3d(0, 0, 0), unit->orientation_);
    double extract_dis = _slice_radius;
    double min_height = _slice_height;
    double max_height = min_height + _slice_thickness;
    CloudType cloud_slice;
    for (size_t i = 0; i < cloud_gravity_aligment.size(); i++) {
      PointType p = cloud_gravity_aligment.points[i];
      if (utils::PointRadius(p) > extract_dis) continue;
      if (p.z > max_height || p.z < min_height) continue;
      // 直接转换到全局
      p.x += unit->position_(0);
      p.y += unit->position_(1);
      p.z += unit->position_(2);
      cloud_slice.push_back(p);
    }

    *top_view_cloud += cloud_slice;
  }

  return top_view_cloud;
}

double LidarMapSession::GetMapUnitTime(int index) {
  if (index < 0 || index >= map_units_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return -1;
  }
  return map_units_[index]->timestamp_;
}

double LidarMapSession::GetMapUnitDistance(int index) {
  if (index < 0 || index >= key_scan_distance_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return -1;
  }

  return key_scan_distance_[index];
}

Eigen::Matrix4d LidarMapSession::GetMapUnitPose(int index) {
  if (index < 0 || index >= key_scan_distance_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return Eigen::Matrix4d::Identity();
  }

  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) = map_units_[index]->orientation_.toRotationMatrix();
  result.block<3, 1>(0, 3) = map_units_[index]->position_;
  return result;
}

Eigen::Vector3d LidarMapSession::GetMapUnitTranslation(int index) {
  if (index < 0 || index >= key_scan_distance_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return Eigen::Vector3d(0, 0, 0);
  }

  return map_units_[index]->position_;
}

bool LidarMapSession::GetMapUnitPose(int index,
                                     Eigen::Vector3d& unit_translation,
                                     Eigen::Quaterniond& unit_rotation) {
  if (index < 0 || index >= key_scan_distance_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return false;
  }

  unit_translation = map_units_[index]->position_;
  unit_rotation = map_units_[index]->orientation_;
  return true;
}

LidarMapUnit::Ptr LidarMapSession::GetMapUnit(int index) {
  if (index < 0 || index >= map_units_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return LidarMapUnit::Ptr();
  }
  return map_units_[index];
}

LidarMapUnit::Ptr LidarMapSession::GetLatestMapUnit() {
  LidarMapUnit::Ptr unit;
  if (!map_units_.empty()) {
    unit = map_units_.back();
  }
  return unit;
}

CloudTypePtr LidarMapSession::GetMapUnitFullCloud(int index) {
  if (index < 0 || index >= map_units_.size()) {
    AWARN_F("[LidarMapSession] Wrong index! index : %i , map_units size :  %i ",
            index, int(map_units_.size()));
    return CloudTypePtr(new CloudType);
  }
  return map_units_[index]->GetFullCloud();
}

void LidarMapSession::Save(std::string _path, double _map_resolution,
                           bool _save_detail_info, bool _save_grid_map) {
  if (!boost::filesystem::exists(_path)) {
    boost::filesystem::create_directories(_path);
  }

  cache_path_ = _path;
  std::string cloud_folder = _path + "/cloud";
  if (_save_detail_info) {
    if (!boost::filesystem::exists(cloud_folder)) {
      boost::filesystem::create_directories(cloud_folder);
    }

    // 保存位姿图数据库
    if (pose_graph_database_ptr_) {
      pose_graph_database_ptr_->Save(_path);
    }
  }

  // 保存LidarMapSession信息
  YAML::Node session_node;
  session_node["session_version"] = session_version_;
  session_node["session_index"] = session_index_;
  session_node["session_name"] = session_name_;
  session_node["map_units_num"] = int(map_units_.size());
  session_node["gnss_alignment"] = gnss_alignment_flag_;
  session_node["gnss_origin_coordinate"] =
      utils::ToVector(gnss_origin_coordinate_);

  // 全局地图
  CloudType global_map;
  // 特征地图
  CloudType feature_map;

  std::vector<Eigen::Vector3d> key_frame_pose_vec;

  // 保存各个map_unit的属性
  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];

    if (_save_detail_info) {
      YAML::Node uint_node;
      uint_node["unit_id"] = unit->unit_id_;
      uint_node["timestamp"] = unit->timestamp_;
      uint_node["bad_flag"] = unit->bad_flag_;
      uint_node["anchor_point_id"] = unit->anchor_point_id_;
      uint_node["position"] = utils::ToVector(unit->position_);
      uint_node["orientation"] = utils::ToVector(unit->orientation_);
      uint_node["ground_coeff"] = utils::ToVector(unit->ground_coeff_);
      uint_node["slice_attribute"] = utils::ToVector(unit->slice_attribute_);

      session_node["map_units"].push_back(uint_node);

      // 保存点云
      std::string cloud_path =
          cloud_folder + "/cloud" + std::to_string(unit->unit_id_) + ".pcd";
      pcl::io::savePCDFileBinaryCompressed(cloud_path, *(unit->full_cloud_));
    }

    key_frame_pose_vec.push_back(unit->position_);

    // 保存特征点云
    // std::string corner_feature_path =
    //     cloud_folder + "/corner" + std::to_string(unit->unit_id_) + ".pcd";
    // pcl::io::savePCDFileBinaryCompressed(corner_feature_path,
    //                                      *(unit->corner_feature_cloud_ds_));
    // std::string surface_feature_path =
    //     cloud_folder + "/surface" + std::to_string(unit->unit_id_) + ".pcd";
    // pcl::io::savePCDFileBinaryCompressed(surface_feature_path,
    //                                      *(unit->surface_feature_cloud_ds_));

    CloudType full_cloud_world;
    pcl::transformPointCloud(*(unit->full_cloud_), full_cloud_world,
                             unit->position_, unit->orientation_);
    // CloudType corner_feature_world;
    // pcl::transformPointCloud(*(unit->corner_feature_cloud_ds_),
    //                          corner_feature_world, unit->position_,
    //                          unit->orientation_);
    // CloudType surface_feature_world;
    // pcl::transformPointCloud(*(unit->surface_feature_cloud_ds_),
    //                          surface_feature_world, unit->position_,
    //                          unit->orientation_);
    global_map += full_cloud_world;
    // feature_map += corner_feature_world;
    // feature_map += surface_feature_world;
  }

  if (_save_detail_info) {
    // 保存 yaml node
    std::string session_info_path = _path + "/session_info.yaml";
    std::ofstream session_info_file(session_info_path.c_str());
    session_info_file << session_node;
    session_info_file.close();
  }

  // 保存全局点云
  std::string global_cloud_path = _path + "/global_cloud.pcd";
  CloudType ds_cloud;
  utils::DownsampleCloudAdapted(global_map, ds_cloud, _map_resolution);
  if (!ds_cloud.empty()) {
    pcl::io::savePCDFileBinaryCompressed(global_cloud_path, ds_cloud);
  }

  // 保存特征点云
  // std::string feature_cloud_path = _path + "/feature_cloud.pcd";
  // CloudType feature_ds_cloud;
  // utils::DownsampleCloudAdapted(feature_map, feature_ds_cloud, 0.1);
  // if (!feature_ds_cloud.empty()) {
  //   pcl::io::savePCDFileBinaryCompressed(feature_cloud_path,
  //   feature_ds_cloud);
  // }

  std::string grid_map_folder = _path + "/grid_map";
  if (_save_grid_map) {
    if (!boost::filesystem::exists(grid_map_folder)) {
      boost::filesystem::create_directories(grid_map_folder);
    }
  }

  // 保存切片点云
  if (_save_detail_info) {
    CloudTypePtr slice_cloud = ExtractTopViewCloud();
    //  utils::DownsampleCloud(*slice_cloud, 0.1);
    std::string slice_cloud_path = _path + "/slice_cloud.pcd";
    if (!slice_cloud->empty()) {
      CloudType slice_ds_cloud;
      utils::DownsampleCloudAdapted(*slice_cloud, slice_ds_cloud,
                                    _map_resolution);
      if (!slice_ds_cloud.empty()) {
        pcl::io::savePCDFileBinaryCompressed(slice_cloud_path, slice_ds_cloud);
      }
    }

    if (_save_grid_map) {
      GridMapGenerator slice_map_generator(slice_cloud, grid_map_folder);
      slice_map_generator.GenerateMap("slice_map");
    }
  }

  // 生成栅格地图
  if (_save_grid_map) {
    GridMapGenerator global_map_generator(ds_cloud.makeShared(),
                                          grid_map_folder);
    global_map_generator.SetKeyFramePoses(key_frame_pose_vec);
    global_map_generator.GenerateMap("global_map");
  }

  AINFO_F("[LidarMapSession] Save Lidar Map Session success!");
}

bool LidarMapSession::Load(std::string _path) {
  if (!boost::filesystem::exists(_path)) {
    AWARN_F("[LidarMapSession] Lidar session folder dose not exit");
    return false;
  }

  std::string cloud_folder = _path + "/cloud";

  std::string session_info_path = _path + "/session_info.yaml";
  YAML::Node session_node;
  try {
    session_node = YAML::LoadFile(session_info_path);
  } catch (...) {
    AERROR_F(
        " [LidarMapSession] The format of config file [ %s ] is wrong, Please "
        "check (e.g. indentation).",
        session_info_path.c_str());
    return false;
  }

  utils::YamlRead<std::string>(session_node, "session_version",
                               session_version_, "");
  utils::YamlRead<int>(session_node, "session_index", session_index_, 0);
  utils::YamlRead<std::string>(session_node, "session_name", session_name_, "");
  int map_units_num;
  utils::YamlRead<int>(session_node, "map_units_num", map_units_num, 0);
  utils::YamlRead<bool>(session_node, "gnss_alignment", gnss_alignment_flag_,
                        false);
  std::vector<double> gnss_origin_coordinate_vec =
      session_node["gnss_origin_coordinate"].as<std::vector<double>>();
  gnss_origin_coordinate_ = Eigen::Vector3d(gnss_origin_coordinate_vec[0],
                                            gnss_origin_coordinate_vec[1],
                                            gnss_origin_coordinate_vec[2]);

  YAML::Node map_units_node =
      utils::YamlSubNodeAbort(session_node, "map_units");

  for (size_t i = 0; i < map_units_node.size(); i++) {
    LidarMapUnit::Ptr new_unit(new LidarMapUnit);
    new_unit->unit_id_ = map_units_node[i]["unit_id"].as<int>();
    new_unit->timestamp_ = map_units_node[i]["timestamp"].as<double>();
    new_unit->bad_flag_ = map_units_node[i]["bad_flag"].as<bool>();
    new_unit->anchor_point_id_ = map_units_node[i]["anchor_point_id"].as<int>();

    std::vector<double> position_vec =
        map_units_node[i]["position"].as<std::vector<double>>();
    std::vector<double> quater_coeff_vec =
        map_units_node[i]["orientation"].as<std::vector<double>>();
    std::vector<double> ground_coeff_vec =
        map_units_node[i]["ground_coeff"].as<std::vector<double>>();
    std::vector<double> slice_attribute_vec =
        map_units_node[i]["slice_attribute"].as<std::vector<double>>();

    new_unit->position_ =
        Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
    new_unit->orientation_ =
        Eigen::Quaterniond(quater_coeff_vec[3], quater_coeff_vec[0],
                           quater_coeff_vec[1], quater_coeff_vec[2]);
    new_unit->ground_coeff_ =
        Eigen::Vector4d(ground_coeff_vec[0], ground_coeff_vec[1],
                        ground_coeff_vec[2], ground_coeff_vec[3]);
    new_unit->slice_attribute_ = Eigen::Vector3d(
        slice_attribute_vec[0], slice_attribute_vec[1], slice_attribute_vec[2]);

    std::string cloud_path =
        cloud_folder + "/cloud" + std::to_string(new_unit->unit_id_) + ".pcd";
    pcl::io::loadPCDFile(cloud_path, *(new_unit->full_cloud_));

    // std::string corner_cloud_path =
    //     cloud_folder + "/corner" + std::to_string(new_unit->unit_id_) +
    //     ".pcd";
    // pcl::io::loadPCDFile(corner_cloud_path,
    //                      *(new_unit->corner_feature_cloud_ds_));
    // std::string surface_cloud_path =
    //     cloud_folder + "/surface" + std::to_string(new_unit->unit_id_) +
    //     ".pcd";
    // pcl::io::loadPCDFile(surface_cloud_path,
    //                      *(new_unit->surface_feature_cloud_ds_));

    AddMapUnit(new_unit);
  }

  if (map_units_.size() != map_units_num) {
    return false;
  }

  // 加载位姿图数据库
  pose_graph_database_ptr_ = PoseGraphDatabase::Ptr(new PoseGraphDatabase);
  if (pose_graph_database_ptr_->Load(_path)) {
    AINFO_F("[LidarMapSession] Load pose graph database success !");
  }

  AINFO_F("[LidarMapSession] Load Map session success");

  cache_path_ = _path;

  return true;
}

Eigen::aligned_vector<Eigen::Vector3d>
LidarMapSession::GetMapUnitsTranslation() {
  Eigen::aligned_vector<Eigen::Vector3d> result;
  result.reserve(map_units_.size());

  for (auto n : map_units_) {
    result.push_back(n->position_);
  }
  return result;
}

void LidarMapSession::SetGNSSOriginCoordinate(double _lon, double _lat,
                                              double _alt) {
  gnss_alignment_flag_ = true;
  gnss_origin_coordinate_ = Eigen::Vector3d(_lon, _lat, _alt);
}

void LidarMapSession::UpdateLENUParam(
    const Eigen::Vector3d& _p_map_in_LENU,
    const Eigen::Quaterniond& _q_map_in_LENU) {
  for (size_t i = 0; i < map_units_.size(); i++) {
    Eigen::Vector3d p_LENU =
        _q_map_in_LENU * map_units_[i]->position_ + _p_map_in_LENU;
    Eigen::Quaterniond q_LENU = _q_map_in_LENU * map_units_[i]->orientation_;

    map_units_[i]->position_ = p_LENU;
    map_units_[i]->orientation_ = q_LENU;

    map_units_translation_cloud_->points[i].x = p_LENU(0);
    map_units_translation_cloud_->points[i].y = p_LENU(1);
    map_units_translation_cloud_->points[i].z = p_LENU(2);
  }
}

void LidarMapSession::UpdateMapUnitPose(
    const int _index, const Eigen::Vector3d& _updated_position,
    const Eigen::Quaterniond& _updated_roation) {
  map_units_[_index]->position_ = _updated_position;
  map_units_[_index]->orientation_ = _updated_roation;

  map_units_translation_cloud_->points[_index].x = _updated_position(0);
  map_units_translation_cloud_->points[_index].y = _updated_position(1);
  map_units_translation_cloud_->points[_index].z = _updated_position(2);
}

void LidarMapSession::UpdateSliceParam(const Eigen::Vector3d& _slice_param,
                                       bool _delta_flag) {
  size_t unit_num = map_units_.size();
  for (size_t i = 0; i < unit_num; i++) {
    if (_delta_flag) {
      map_units_[i]->slice_attribute_ += _slice_param;
    } else {
      map_units_[i]->slice_attribute_ = _slice_param;
    }
  }
}

void LidarMapSession::UpdateMapYaw(double _yaw) {
  Eigen::Quaterniond map_q(
      Eigen::AngleAxisd(_yaw * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
  size_t unit_num = map_units_.size();
  for (size_t i = 0; i < unit_num; i++) {
    map_units_[i]->position_ = map_q * map_units_[i]->position_;
    map_units_[i]->orientation_ = map_q * map_units_[i]->orientation_;

    map_units_translation_cloud_->points[i].x = map_units_[i]->position_(0);
    map_units_translation_cloud_->points[i].y = map_units_[i]->position_(1);
    map_units_translation_cloud_->points[i].z = map_units_[i]->position_(2);
  }
}

void LidarMapSession::UpdateMapYaw(const Eigen::Quaterniond& _rotation) {
  size_t unit_num = map_units_.size();
  for (size_t i = 0; i < unit_num; i++) {
    map_units_[i]->position_ = _rotation * map_units_[i]->position_;
    map_units_[i]->orientation_ = _rotation * map_units_[i]->orientation_;

    map_units_translation_cloud_->points[i].x = map_units_[i]->position_(0);
    map_units_translation_cloud_->points[i].y = map_units_[i]->position_(1);
    map_units_translation_cloud_->points[i].z = map_units_[i]->position_(2);
  }
}

void LidarMapSession::SavePoseAsTumFormat(std::string _file_name) {
  std::ofstream outfile;
  outfile.open(_file_name);

  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];

    outfile.precision(18);
    outfile << unit->timestamp_ << " ";
    outfile.precision(5);
    outfile << unit->position_(0) << " " << unit->position_(1) << " "
            << unit->position_(2) << " " << unit->orientation_.x() << " "
            << unit->orientation_.y() << " " << unit->orientation_.z() << " "
            << unit->orientation_.w() << "\n";
  }

  outfile.close();
}

void LidarMapSession::SaveCloudAsKAISTFormat(std::string _cloud_folder) {
  if (!boost::filesystem::exists(_cloud_folder)) {
    boost::filesystem::create_directories(_cloud_folder);
  }
  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];

    int64_t cloud_timestamp_ns = unit->timestamp_ * 1e9;
    std::string cloud_file_name =
        _cloud_folder + "/" + std::to_string(cloud_timestamp_ns) + ".pcd";
    pcl::io::savePCDFileBinaryCompressed(cloud_file_name, *(unit->full_cloud_));
  }
}

CloudTypePtr LidarMapSession::GetGlobalMap(double _resolution) {
  CloudTypePtr global_cloud(new CloudType);

  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];

    CloudType full_cloud_world;
    pcl::transformPointCloud(*(unit->full_cloud_), full_cloud_world,
                             unit->position_, unit->orientation_);
    *global_cloud += full_cloud_world;
  }

  CloudTypePtr ds_cloud(new CloudType);
  utils::DownsampleCloudAdapted(*global_cloud, *ds_cloud, _resolution);

  return ds_cloud;
}

CloudTypePtr LidarMapSession::GetKeyFramePoseCloud() {
  CloudTypePtr keyframe_cloud(new CloudType);

  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];

    PointType pose_point;
    pose_point.x = unit->position_(0);
    pose_point.y = unit->position_(1);
    pose_point.z = unit->position_(2);
    keyframe_cloud->push_back(pose_point);
  }
  return keyframe_cloud;
}

PoseGraph LidarMapSession::ExportPoseGraph() {
  PoseGraph graph;
  for (size_t i = 0; i < map_units_.size(); i++) {
    LidarMapUnit::Ptr unit = map_units_[i];

    GraphVertex vertex;
    vertex.id = unit->unit_id_;
    vertex.position = unit->position_;
    vertex.orientation = unit->orientation_;
    vertex.valid_flag = true;
    graph.vertices.push_back(vertex);
  }

  if (pose_graph_database_ptr_) {
    for (int lc_id : pose_graph_database_ptr_->loop_closure_info_id_vec_) {
      graph.loop_closure_edegs_.push_back(
          pose_graph_database_ptr_->loop_closure_constraint_info_[lc_id]);
    }

    for (int gc_id : pose_graph_database_ptr_->gnss_constraint_id_vec_) {
      graph.gnss_edges.push_back(
          pose_graph_database_ptr_->gnss_constraint_info_[gc_id]);
    }
  }

  return graph;
}

bool LidarMapSession::GetLocalMatchInformation(int _lc_id,
                                               LocalMatchInformation& _info) {
  if (!pose_graph_database_ptr_) return false;

  auto lc_iter =
      pose_graph_database_ptr_->loop_closure_constraint_info_.find(_lc_id);
  if (lc_iter ==
      pose_graph_database_ptr_->loop_closure_constraint_info_.end()) {
    return false;
  }

  // 获取相关点云
  CloudTypePtr target_cloud(new CloudType);
  Eigen::Matrix4d target_pose;
  FindNearbyKeyFrames(lc_iter->second.front_id, target_cloud, 10, target_pose);

  CloudTypePtr source_cloud(new CloudType);
  Eigen::Matrix4d source_pose;
  FindNearbyKeyFrames(lc_iter->second.back_id, source_cloud, 0, source_pose);

  _info.loop_closure_id = _lc_id;
  _info.target_cloud = *target_cloud;
  _info.source_cloud = *source_cloud;
  _info.target_pose_in_map = target_pose;
  _info.source_pose_in_map = source_pose;
  _info.relative_pose = utils::ToMatrix4d(lc_iter->second.delta_position,
                                          lc_iter->second.delta_rotation);
  _info.score = lc_iter->second.reg_score;

  return true;
}

void LidarMapSession::DrawLoopClosureThreeView(
    const CloudTypePtr& _target_cloud, const Eigen::Matrix4d& _target_pose,
    const CloudTypePtr& _source_cloud, const Eigen::Matrix4d& _source_pose,
    cv::Mat& _up_view, cv::Mat& _front_view, cv::Mat& _left_view) {
  Eigen::Matrix4d target_pose_in_local = _target_pose;
  target_pose_in_local.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix4d source_pose_in_local =
      target_pose_in_local * (_target_pose.inverse() * _source_pose);

  CloudTypePtr target_cloud_in_local(new CloudType);
  pcl::transformPointCloud(*_target_cloud, *target_cloud_in_local,
                           target_pose_in_local);

  CloudTypePtr source_cloud_in_local(new CloudType);
  pcl::transformPointCloud(*_source_cloud, *source_cloud_in_local,
                           source_pose_in_local);

  // ROI 滤波
  double roi_radius = 25;
  double grid_resolution = 0.1;

  CloudType drawed_cloud;
  int target_cloud_size = target_cloud_in_local->size();
  for (int i = 0; i < target_cloud_size; i++) {
    PointType p = target_cloud_in_local->points[i];
    if (p.x > roi_radius || p.x < -roi_radius || p.y > roi_radius ||
        p.y < -roi_radius)
      continue;
    p.intensity = 0;
    drawed_cloud.push_back(p);
  }

  int source_cloud_size = source_cloud_in_local->size();
  for (int i = 0; i < source_cloud_size; i++) {
    PointType p = source_cloud_in_local->points[i];
    if (p.x > roi_radius || p.x < -roi_radius || p.y > roi_radius ||
        p.y < -roi_radius)
      continue;
    p.intensity = 1;
    drawed_cloud.push_back(p);
  }

  // 画图
  // 三视图画布大小
  float len_x[3], len_y[3];
  int width[3], height[3];

  drawed_cloud.is_dense = false;
  Eigen::Vector4f min_cloud;
  Eigen::Vector4f max_cloud;

  pcl::getMinMax3D<PointType>(drawed_cloud, min_cloud, max_cloud);
  min_cloud(0) -= 1;
  min_cloud(1) -= 1;
  min_cloud(2) -= 1;
  max_cloud(0) += 1;
  max_cloud(1) += 1;
  max_cloud(2) += 1;

  // 上视图
  len_x[0] = max_cloud(0) - min_cloud(0);
  len_y[0] = max_cloud(1) - min_cloud(1);

  // 前视图
  len_x[1] = max_cloud(1) - min_cloud(1);
  len_y[1] = max_cloud(2) - min_cloud(2);

  // 左视图
  len_x[2] = max_cloud(0) - min_cloud(0);
  len_y[2] = max_cloud(2) - min_cloud(2);

  Eigen::Vector3d cloud_origin_position =
      Eigen::Vector3d(min_cloud(0), min_cloud(1), min_cloud(2));

  cv::Mat three_view[3];

  for (int i = 0; i < 3; i++) {
    // x轴方向是width y方向是height
    width[i] = len_x[i] / grid_resolution;
    height[i] = len_y[i] / grid_resolution;

    // 重新设置成白布
    three_view[i] =
        cv::Mat(height[i], width[i], CV_8UC3, cv::Vec3b(255, 255, 255));
  }

  int drawed_cloud_size = drawed_cloud.size();
  for (int i = 0; i < drawed_cloud_size; i++) {
    // 上视图
    int col = int((drawed_cloud.points[i].x - cloud_origin_position.x()) /
                  grid_resolution);
    int row =
        height[0] - int((drawed_cloud.points[i].y - cloud_origin_position.y()) /
                        grid_resolution);
    if (drawed_cloud.points[i].intensity == 0) {
      three_view[0].at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
    } else {
      three_view[0].at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
    }

    // 前视图
    col = int((drawed_cloud.points[i].y - cloud_origin_position.y()) /
              grid_resolution);
    row =
        height[1] - int((drawed_cloud.points[i].z - cloud_origin_position.z()) /
                        grid_resolution);
    if (drawed_cloud.points[i].intensity == 0) {
      three_view[1].at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
    } else {
      three_view[1].at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
    }

    // // 左视图
    col = int((drawed_cloud.points[i].x - cloud_origin_position.x()) /
              grid_resolution);
    row =
        height[2] - int((drawed_cloud.points[i].z - cloud_origin_position.z()) /
                        grid_resolution);
    if (drawed_cloud.points[i].intensity == 0) {
      three_view[2].at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
    } else {
      three_view[2].at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
    }
  }

  _up_view = three_view[0].clone();
  _front_view = three_view[1].clone();
  _left_view = three_view[2].clone();
}

bool LidarMapSession::SaveLoopClosureThreeView(std::string _cache_path) {
  if (!pose_graph_database_ptr_) return false;
  if (pose_graph_database_ptr_->LoopClosureConstraintEmpty()) return false;

  if (!boost::filesystem::exists(_cache_path)) {
    boost::filesystem::create_directories(_cache_path);
  }

  std::string loop_closure_path = _cache_path + "/loop_closure";
  if (!boost::filesystem::exists(loop_closure_path)) {
    boost::filesystem::create_directories(loop_closure_path);
  }

  for (size_t i = 0;
       i < pose_graph_database_ptr_->loop_closure_info_id_vec_.size(); i++) {
    int loop_closure_id =
        pose_graph_database_ptr_->loop_closure_info_id_vec_[i];
    auto lc = pose_graph_database_ptr_
                  ->loop_closure_constraint_info_[loop_closure_id];

    cv::Mat up_view, front_view, left_view;
    DrawLoopClosureThreeView(
        GetMapUnitFullCloud(lc.front_id), GetMapUnitPose(lc.front_id),
        GetMapUnitFullCloud(lc.back_id), GetMapUnitPose(lc.back_id), up_view,
        front_view, left_view);

    std::string lc_path =
        loop_closure_path + "/lc-" + std::to_string(loop_closure_id);
    if (!boost::filesystem::exists(lc_path)) {
      boost::filesystem::create_directories(lc_path);
    }

    cv::imwrite(lc_path + "/up.png", up_view);
    cv::imwrite(lc_path + "/front.png", front_view);
    cv::imwrite(lc_path + "/left.png", left_view);
  }

  return true;
}

bool LidarMapSession::SaveLoopClosureCloud() {
  if (!boost::filesystem::exists(cache_path_)) {
    AWARN_F("[LidarMapSession] Please set lidar map session cache path. ");
    return false;
  }

  std::string loop_closure_path = cache_path_ + "/loop_closure";

  if (!boost::filesystem::exists(loop_closure_path)) {
    boost::filesystem::create_directories(loop_closure_path);
  }

  for (size_t i = 0;
       i < pose_graph_database_ptr_->loop_closure_info_id_vec_.size(); i++) {
    int loop_closure_id =
        pose_graph_database_ptr_->loop_closure_info_id_vec_[i];
    auto lc = pose_graph_database_ptr_
                  ->loop_closure_constraint_info_[loop_closure_id];

    // 获取相关点云
    CloudTypePtr target_cloud(new CloudType);
    Eigen::Matrix4d target_pose;
    FindNearbyKeyFrames(lc.front_id, target_cloud, 10, target_pose);

    CloudTypePtr source_cloud(new CloudType);
    Eigen::Matrix4d source_pose;
    FindNearbyKeyFrames(lc.back_id, source_cloud, 0, source_pose);

    Eigen::Matrix4d target_pose_in_local = target_pose;
    target_pose_in_local.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix4d source_pose_in_local =
        target_pose_in_local *
        (utils::ToMatrix4d(lc.delta_position, lc.delta_rotation));

    CloudTypePtr target_cloud_in_local(new CloudType);
    pcl::transformPointCloud(*target_cloud, *target_cloud_in_local,
                             target_pose_in_local);

    CloudTypePtr source_cloud_in_local(new CloudType);
    pcl::transformPointCloud(*source_cloud, *source_cloud_in_local,
                             source_pose_in_local);

    utils::DownsampleCloud(*target_cloud_in_local, 0.1);
    utils::DownsampleCloud(*source_cloud_in_local, 0.1);

    std::string lc_path =
        loop_closure_path + "/lc-" + std::to_string(loop_closure_id);
    if (!boost::filesystem::exists(lc_path)) {
      boost::filesystem::create_directories(lc_path);
    }

    pcl::io::savePCDFileBinaryCompressed(lc_path + "/target_cloud.pcd",
                                         *target_cloud_in_local);
    pcl::io::savePCDFileBinaryCompressed(lc_path + "/source_cloud.pcd",
                                         *source_cloud_in_local);
  }

  return true;
}

bool LidarMapSession::FindNearbyKeyFrames(int _key_frame_id,
                                          CloudTypePtr _cloud,
                                          const int _search_num) {
  _cloud->clear();
  for (int i = -_search_num; i <= _search_num; ++i) {
    int key_nearby = _key_frame_id + i;
    LidarMapUnit::Ptr map_unit_nearby = GetMapUnit(key_nearby);
    if (!map_unit_nearby) continue;
    CloudType global_cloud;
    pcl::transformPointCloud(*map_unit_nearby->full_cloud_, global_cloud,
                             map_unit_nearby->position_,
                             map_unit_nearby->orientation_);

    *_cloud += global_cloud;
  }
  if (_cloud->empty()) return false;
  return true;
}

bool LidarMapSession::FindNearbyKeyFrames(int _key_frame_id,
                                          CloudTypePtr _cloud,
                                          const int _search_num,
                                          Eigen::Matrix4d& _cloud_pose) {
  _cloud->clear();
  LidarMapUnit::Ptr cur_map_unit = GetMapUnit(_key_frame_id);
  if (!cur_map_unit) return false;
  _cloud_pose =
      utils::ToMatrix4d(cur_map_unit->position_, cur_map_unit->orientation_);

  *_cloud += *(cur_map_unit->full_cloud_);

  for (int i = -_search_num; i <= _search_num; ++i) {
    if (i == 0) continue;

    int key_nearby = _key_frame_id + i;
    if (key_nearby < 0 || key_nearby >= GetMapUnitSize()) continue;

    LidarMapUnit::Ptr map_unit_nearby = GetMapUnit(key_nearby);
    if (!map_unit_nearby) continue;

    Eigen::Matrix4d local_pose =
        _cloud_pose.inverse() *
        utils::ToMatrix4d(map_unit_nearby->position_,
                          map_unit_nearby->orientation_);

    CloudTypePtr local_cloud(new CloudType());
    pcl::transformPointCloud(*map_unit_nearby->full_cloud_, *local_cloud,
                             local_pose);

    *_cloud += *local_cloud;
  }

  if (_cloud->empty()) return false;
  return true;
}

}  // namespace multi_sensor_mapping
