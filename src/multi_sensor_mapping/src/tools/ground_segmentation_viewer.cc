#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "config.h"
#include "multi_sensor_mapping/frontend/lidar/patchwork_pp.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/map/lidar_map_unit.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation_viewer");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  bool debug_mode = false;
  double ds_size;
  std::string session_path;
  std::string config_path;
  nh_param.param<std::string>("session_path", session_path, "");
  nh_param.param<std::string>("config_path", config_path, "");
  nh_param.param<bool>("debug_mode", debug_mode, false);
  nh_param.param<double>("ds_size", ds_size, 0.01);

  ros::Publisher pub_aligned_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 1);
  ros::Publisher pub_init_ground_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("init_ground_cloud", 1);
  ros::Publisher pub_init_nonground_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/init_nonground_cloud", 1);
  ros::Publisher pub_ground_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
  ros::Publisher pub_nonground_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/nonground_cloud", 1);

  // 加载地图
  LidarMapSession map_session;
  if (!map_session.Load(session_path)) {
    std::cout << "Load map session error !" << std::endl;
  }

  // 加载patchwork参数
  PatchWorkParams::Ptr param(new PatchWorkParams);
  std::string source_dir = MSM_SOURCE_DIR;
  std::string param_path = source_dir + "/config/" + config_path;
  param->Load(param_path);

  // 初始化Patchwork
  PatchworkPP patchwork(param);

  CloudType ground_map;
  CloudType non_ground_map;

  int map_unit_size = map_session.GetMapUnitSize();
  for (int i = 0; i < map_unit_size; i++) {
    std::shared_ptr<LidarMapUnit> map_unit = map_session.GetMapUnit(i);

    // 重力对齐点云
    CloudType aligned_cloud;
    Eigen::Matrix4f aligned_pose = Eigen::Matrix4f::Identity();
    aligned_pose.block<3, 3>(0, 0) =
        map_unit->orientation_.toRotationMatrix().template cast<float>();
    pcl::transformPointCloud(*map_unit->full_cloud_, aligned_cloud,
                             aligned_pose);

    // 初始化地面提取
    CloudType init_ground_cloud;
    CloudType init_nonground_cloud;
    patchwork.EstimateGround(aligned_cloud, init_ground_cloud,
                             init_nonground_cloud);

    CloudType ground_cloud;
    CloudType nonground_cloud;
    patchwork.EstimateGround(aligned_cloud, init_ground_cloud, ground_cloud,
                             nonground_cloud, 20);

    Eigen::Matrix4f translation_matrix = Eigen::Matrix4f::Identity();
    translation_matrix.block<3, 1>(0, 3) = map_unit->position_.cast<float>();

    CloudType ground_cloud_in_map;
    CloudType nonground_cloud_in_map;

    pcl::transformPointCloud(ground_cloud, ground_cloud_in_map,
                             translation_matrix);
    pcl::transformPointCloud(nonground_cloud, nonground_cloud_in_map,
                             translation_matrix);

    ground_map += ground_cloud_in_map;
    non_ground_map += nonground_cloud_in_map;

    if (debug_mode) {
      // 点云发布
      sensor_msgs::PointCloud2 aligned_cloud_msg;
      pcl::toROSMsg(aligned_cloud, aligned_cloud_msg);
      aligned_cloud_msg.header.frame_id = "map";

      sensor_msgs::PointCloud2 init_ground_cloud_msg;
      pcl::toROSMsg(init_ground_cloud, init_ground_cloud_msg);
      init_ground_cloud_msg.header.frame_id = "map";

      sensor_msgs::PointCloud2 init_nonground_cloud_msg;
      pcl::toROSMsg(init_nonground_cloud, init_nonground_cloud_msg);
      init_nonground_cloud_msg.header.frame_id = "map";

      sensor_msgs::PointCloud2 ground_cloud_msg;
      pcl::toROSMsg(ground_cloud, ground_cloud_msg);
      ground_cloud_msg.header.frame_id = "map";

      sensor_msgs::PointCloud2 nonground_cloud_msg;
      pcl::toROSMsg(nonground_cloud, nonground_cloud_msg);
      nonground_cloud_msg.header.frame_id = "map";

      pub_aligned_cloud.publish(aligned_cloud_msg);
      pub_ground_cloud.publish(ground_cloud_msg);
      pub_nonground_cloud.publish(nonground_cloud_msg);
      pub_init_ground_cloud.publish(init_ground_cloud_msg);
      pub_init_nonground_cloud.publish(init_nonground_cloud_msg);

      std::getchar();
    }
    if (!ros::ok()) {
      break;
    }
  }

  std::string ground_cloud_path = session_path + "/ground_cloud.pcd";
  std::string non_ground_cloud_path = session_path + "/nonground_cloud.pcd";
  CloudType ground_cloud_ds;
  utils::DownsampleCloudAdapted(ground_map, ground_cloud_ds, ds_size);
  CloudType nonground_cloud_ds;
  utils::DownsampleCloudAdapted(non_ground_map, nonground_cloud_ds, ds_size);

  pcl::io::savePCDFileBinaryCompressed(ground_cloud_path, ground_cloud_ds);
  pcl::io::savePCDFileBinaryCompressed(non_ground_cloud_path,
                                       nonground_cloud_ds);

  return 0;
}