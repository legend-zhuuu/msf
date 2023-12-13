
#ifndef MSM_LIDAR_MAP_SESSION_H
#define MSM_LIDAR_MAP_SESSION_H

#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "multi_sensor_mapping/utils/utils_eigen.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

class LidarMapUnit;
class PoseGraphDatabase;
struct PoseGraph;
struct LocalMatchInformation;

/**
 * @brief The LidarMapSession class 激光建图任务
 */
class LidarMapSession {
 public:
  typedef std::shared_ptr<LidarMapSession> Ptr;

  /**
   * @brief LidarMapSession 构造函数
   */
  LidarMapSession();

  /**
   * @brief LidarMapSession 构造函数
   * @param _session_id
   * @param _session_name
   */
  LidarMapSession(int _session_id, std::string _session_name);

  /**
   * @brief Set the Cache Path object 设置缓存路径
   *
   * @param _path
   */
  void SetCachePath(std::string _path);

  /**
   * @brief AddMapUnit 添加新的地图元
   * @param map_unit
   */
  void AddMapUnit(std::shared_ptr<LidarMapUnit> _map_unit);

  /**
   * @brief ExtractSurroundingKeyFrames 获取当前关键帧周围点云
   * @param cur_timestamp
   * @param key_frame_search_radius
   * @param surrounding_corner_cloud
   * @param surrounding_surface_cloud
   * @return
   */
  int ExtractSurroundingKeyFrames(double _cur_timestamp,
                                  double _key_frame_search_radius,
                                  CloudTypePtr& _surrounding_corner_cloud,
                                  CloudTypePtr& _surrounding_surface_cloud);

  /**
   * @brief ExtractTopViewCloud 提取俯视图点云
   * @param _delta_slice_radius 切片半径变化量
   * @param _delta_slice_height 切片高度变化量
   * @param _delta_slice_thickness 切片厚度变化量
   * @return
   */
  CloudTypePtr ExtractTopViewCloud(double _delta_slice_radius = 0,
                                   double _delta_slice_height = 0,
                                   double _delta_slice_thickness = 0);

  /**
   * @brief ExtractTopViewCloud2 提取俯视图点云
   *
   * @param _slice_radius
   * @param _slice_height
   * @param _slice_thickness
   * @return CloudTypePtr
   */
  CloudTypePtr ExtractTopViewCloud2(double _slice_radius, double _slice_height,
                                    double _slice_thickness);

  /**
   * @brief GetMapUnitTime 获取时间戳
   * @param _index
   * @return
   */
  double GetMapUnitTime(int _index);

  /**
   * @brief GetMapUnitDistance 获取距离
   * @param _index
   * @return
   */
  double GetMapUnitDistance(int _index);

  /**
   * @brief GetMapUnitPose 获取姿态
   * @param _index
   * @return
   */
  Eigen::Matrix4d GetMapUnitPose(int _index);

  /**
   * @brief GetMapUnitTranslation 获取平移量
   * @param _index
   * @return
   */
  Eigen::Vector3d GetMapUnitTranslation(int _index);

  /**
   * @brief GetMapUnitPose 获取姿态
   * @param _index
   * @param _unit_translation
   * @param _unit_rotation
   * @return
   */
  bool GetMapUnitPose(int _index, Eigen::Vector3d& _unit_translation,
                      Eigen::Quaterniond& _unit_rotation);

  /**
   * @brief GetMapUnit 获取地图元
   * @param _index
   * @return
   */
  std::shared_ptr<LidarMapUnit> GetMapUnit(int _index);

  /**
   * @brief GetLatestMapUnit 获取最新的地图元
   *
   * @return std::shared_ptr<LidarMapUnit>
   */
  std::shared_ptr<LidarMapUnit> GetLatestMapUnit();

  /**
   * @brief 获取指定MapUnit的全部点云
   *
   * @param _index
   * @return CloudTypePtr
   */
  CloudTypePtr GetMapUnitFullCloud(int _index);

  /**
   * @brief 保存地图
   *
   * @param _path
   * @param _map_resolution
   * @param _save_detail_info
   * @param _save_grid_map
   */
  void Save(std::string _path, double _map_resolution = 0.1,
            bool _save_detail_info = true, bool _save_grid_map = true);

  /**
   * @brief Load 加载
   * @param _path
   * @return
   */
  bool Load(std::string _path);

  /**
   * @brief GetNumMapUnits 获取关键帧数量
   * @return
   */
  inline int GetMapUnitSize() { return map_units_.size(); }

  /**
   * @brief GetSessionId 获取建图任务的ID
   * @return
   */
  inline int GetSessionId() { return session_index_; }

  /**
   * @brief GetSessionName 获取建图任务的名字
   * @return
   */
  inline std::string GetSessionName() { return session_name_; }

  /**
   * @brief Empty 是否为空
   * @return
   */
  inline bool Empty() { return map_units_.empty(); }

  /**
   * @brief Update 地图元数据库是否更新
   * @return
   */
  inline bool Update() { return update_flag_; }

  /**
   * @brief GetMapUnitsTranslation 获取地图元位姿点云
   * @return
   */
  inline CloudTypePtr GetMapUnitsTranslationCloud() {
    return map_units_translation_cloud_;
  }

  /**
   * @brief Get the Cache Path object 获取缓存路径
   *
   * @return std::string
   */
  inline std::string GetCachePath() { return cache_path_; }

  /**
   * @brief GetMapUnitsTrans 获取地图元位姿平移
   * @return
   */
  Eigen::aligned_vector<Eigen::Vector3d> GetMapUnitsTranslation();

  /**
   * @brief SetGNSSOriginCoordinate
   * @param _lon
   * @param _lat
   * @param _alt
   */
  void SetGNSSOriginCoordinate(double _lon, double _lat, double _alt);

  /**
   * @brief UpdateLENUParam 将map_unit坐标转换到LENU坐标系
   * @param _p_map_in_LENU
   * @param _q_map_in_LENU
   */
  void UpdateLENUParam(const Eigen::Vector3d& _p_map_in_LENU,
                       const Eigen::Quaterniond& _q_map_in_LENU);

  /**
   * @brief UpdateMapUnitPose 更新map unit的姿态
   * @param _index
   * @param _updated_position
   * @param _updated_roation
   */
  void UpdateMapUnitPose(const int _index,
                         const Eigen::Vector3d& _updated_position,
                         const Eigen::Quaterniond& _updated_roation);

  /**
   * @brief 更新切片参数
   *
   * @param _slice_param
   */
  void UpdateSliceParam(const Eigen::Vector3d& _slice_param,
                        bool _delta_flag = true);

  /**
   * @brief 更新地图yaw角
   *
   * @param _yaw
   */
  void UpdateMapYaw(double _yaw);

  /**
   * @brief 更新地图yaw角
   *
   * @param _rotation
   */
  void UpdateMapYaw(const Eigen::Quaterniond& _rotation);

  /**
   * @brief SavePoseAsTumFormat 保存姿态(TUM格式)
   * @param _file_name
   */
  void SavePoseAsTumFormat(std::string _file_name);

  /**
   * @brief SaveCloudAsKAISTFormat 保存点云(时间戳命名)
   * @param _cloud_folder
   */
  void SaveCloudAsKAISTFormat(std::string _cloud_folder);

  /**
   * @brief GetGlobalMap 获取地图
   * @return
   */
  CloudTypePtr GetGlobalMap(double _resolution = 0.1);

  /**
   * @brief Get the Key Frame Pose Cloud object 获取关键帧位置点云
   *
   * @return CloudTypePtr
   */
  CloudTypePtr GetKeyFramePoseCloud();

  /**
   * @brief 导出位姿图
   *
   * @return PoseGraph
   */
  PoseGraph ExportPoseGraph();

  /**
   * @brief Get the Local Match Information object 获取局部匹配信息
   *
   * @param _lc_id
   * @return LocalMatchInformation
   */
  bool GetLocalMatchInformation(int _lc_id, LocalMatchInformation& _info);

  /**
   * @brief 绘制闭环点云三视图
   *
   * @param _target_cloud
   * @param _target_pose
   * @param _source_cloud
   * @param _source_pose
   * @return std::vector<cv::Mat>
   */
  void DrawLoopClosureThreeView(const CloudTypePtr& _target_cloud,
                                const Eigen::Matrix4d& _target_pose,
                                const CloudTypePtr& _source_cloud,
                                const Eigen::Matrix4d& _source_pose,
                                cv::Mat& _up_view, cv::Mat& _front_view,
                                cv::Mat& _left_view);

  /**
   * @brief 保存三视图
   *
   * @param _cache_path
   * @return true
   * @return false
   */
  bool SaveLoopClosureThreeView(std::string _cache_path);

  /**
   * @brief 保存闭环点云
   *
   * @return true
   * @return false
   */
  bool SaveLoopClosureCloud();

  /**
   * @brief FindNearbyKeyFrames 寻找相邻的关键帧(转化到全局坐标系下)
   * @param _key_frame_id
   * @param _cloud
   * @param _search_num
   * @return
   */
  bool FindNearbyKeyFrames(int _key_frame_id, CloudTypePtr _cloud,
                           const int _search_num);

  /**
   * @brief FindNearbyKeyFrames 寻找相邻的关键帧(转换到局部坐标系下)
   * @param _key_frame_id
   * @param _cloud
   * @param _search_num
   * @param _cloud_pose
   * @return
   */
  bool FindNearbyKeyFrames(int _key_frame_id, CloudTypePtr _cloud,
                           const int _search_num, Eigen::Matrix4d& _cloud_pose);

 public:
  /// @brief 是否为基任务(针对于多任务系统来说)
  bool is_base_session_;

  /// @brief 位姿图数据库(与PoseGraphManager共享)
  std::shared_ptr<PoseGraphDatabase> pose_graph_database_ptr_;

 private:
  /// @brief 地图版本(主要用于版本兼容)
  std::string session_version_;
  /// @brief 任务索引
  int session_index_;
  /// @brief 任务名称
  std::string session_name_;
  /// @brief 地图元数据库
  std::vector<std::shared_ptr<LidarMapUnit>> map_units_;
  /// @brief 数据库是否更新
  bool update_flag_;
  /// @brief 保存每个关键帧走过的距离
  std::vector<double> key_scan_distance_;
  /// @brief 关键帧位置(用点云存储，方便kdtree搜索)
  CloudTypePtr map_units_translation_cloud_;
  /// @brief 关键帧降采样
  pcl::VoxelGrid<PointType> surrounding_key_poses_filter_;

  /// @brief 缓存路径
  std::string cache_path_;

  /// @brief GNSS坐标对齐标志位
  bool gnss_alignment_flag_;
  /// @brief 地图起始点坐标
  Eigen::Vector3d gnss_origin_coordinate_;
};

}  // namespace multi_sensor_mapping

#endif
