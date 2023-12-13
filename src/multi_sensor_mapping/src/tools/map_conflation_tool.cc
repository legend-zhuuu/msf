#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/filesystem.hpp>

#include "multi_sensor_mapping/adjust_cloud_panel_cmd.h"
#include "multi_sensor_mapping/frontend/lidar/ndt_scan_matcher.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

using namespace multi_sensor_mapping;

bool update_pose_flag = false;
bool match_flag = false;
Eigen::Matrix4f cloud_mat = Eigen::Matrix4f::Identity();

void AdjustCloudCmdHandler(
    const multi_sensor_mapping::adjust_cloud_panel_cmd::ConstPtr& _panel_msg) {
  if (_panel_msg->cmd == "ADJ") {
    cloud_mat(0, 3) = _panel_msg->x;
    cloud_mat(1, 3) = _panel_msg->y;
    cloud_mat(2, 3) = _panel_msg->z;

    Eigen::Quaternionf q(Eigen::AngleAxisf(_panel_msg->yaw * M_PI / 180.0,
                                           Eigen::Vector3f(0, 0, 1)) *
                         Eigen::AngleAxisf(_panel_msg->pitch * M_PI / 180.0,
                                           Eigen::Vector3f(0, 1, 0)) *
                         Eigen::AngleAxisf(_panel_msg->roll * M_PI / 180.0,
                                           Eigen::Vector3f(1, 0, 0)));
    cloud_mat.block<3, 3>(0, 0) = q.toRotationMatrix();

    update_pose_flag = true;
  } else if (_panel_msg->cmd == "CONFIRM") {
    match_flag = true;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_conflation_tool");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  ros::Subscriber sub_panel_cmd =
      nh.subscribe<multi_sensor_mapping::adjust_cloud_panel_cmd>(
          "/adjust_panel_cmd", 1, AdjustCloudCmdHandler);

  ros::Publisher pub_target_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);
  ros::Publisher pub_source_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/source_cloud", 1);

  std::string target_cloud_path;
  std::string source_cloud_path;
  nh_param.param<std::string>("target_cloud_path", target_cloud_path, "");
  nh_param.param<std::string>("source_cloud_path", source_cloud_path, "");

  boost::filesystem::path target_path(target_cloud_path);

  // 加载点云地图
  CloudTypePtr target_cloud(new CloudType);
  CloudTypePtr source_cloud(new CloudType);

  pcl::io::loadPCDFile(target_cloud_path, *target_cloud);
  pcl::io::loadPCDFile(source_cloud_path, *source_cloud);

  sensor_msgs::PointCloud2 target_cloud_msg;
  sensor_msgs::PointCloud2 source_cloud_msg;

  pcl::toROSMsg(*target_cloud, target_cloud_msg);
  pcl::toROSMsg(*source_cloud, source_cloud_msg);
  target_cloud_msg.header.frame_id = "map";
  source_cloud_msg.header.frame_id = "map";

  CloudTypePtr source_cloud_transformed(new CloudType);

  int cnt = 0;
  ros::Rate rate(3);
  while (ros::ok()) {
    if (update_pose_flag) {
      pcl::transformPointCloud(*source_cloud, *source_cloud_transformed,
                               cloud_mat);
      pcl::toROSMsg(*source_cloud_transformed, source_cloud_msg);
      source_cloud_msg.header.frame_id = "map";
      update_pose_flag = false;
    }

    if (match_flag) {
      Eigen::Matrix4f estimate_pose;
      NdtScanMatcher ndt_matcher(2.0, 1.0, 4);
      ndt_matcher.SetTargetCloud(target_cloud);
      ndt_matcher.Match(source_cloud, cloud_mat, estimate_pose);

      pcl::transformPointCloud(*source_cloud, *source_cloud_transformed,
                               estimate_pose);
      pcl::toROSMsg(*source_cloud_transformed, source_cloud_msg);
      source_cloud_msg.header.frame_id = "map";

      match_flag = false;
    }

    if (cnt++ % 30 == 0) {
      pub_target_cloud.publish(target_cloud_msg);
    }
    pub_source_cloud.publish(source_cloud_msg);

    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "Save cloud" << std::endl;

  CloudTypePtr sum_cloud(new CloudType);
  *sum_cloud += *target_cloud;
  *sum_cloud += *source_cloud_transformed;

  CloudTypePtr sum_cloud_ds(new CloudType);
  utils::DownsampleCloudAdapted(*sum_cloud, *sum_cloud_ds, 0.1);

  std::string save_path = target_path.parent_path().string() + "/merge_map.pcd";
  pcl::io::savePCDFileBinaryCompressed(save_path, *sum_cloud_ds);

  std::cout << "Save cloud at : " << save_path << std::endl;

  return 0;
}