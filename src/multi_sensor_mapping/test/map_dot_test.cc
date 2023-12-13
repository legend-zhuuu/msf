#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"

void LoadPoints(std::string _path, std::vector<Eigen::Vector2f>& _dot_points) {
  YAML::Node points_node;
  try {
    points_node = YAML::LoadFile(_path);
  } catch (...) {
    AERROR_F(
        "The format of config file [ %s ] is wrong, Please check (e.g. "
        "indentation).",
        _path.c_str());
    return;
  }

  std::cout << points_node.size() << std::endl;

  for (size_t i = 0; i < points_node.size(); i++) {
    YAML::Node point_node = points_node[i];

    float x = point_node["x"].as<float>();
    float y = point_node["y"].as<float>();
    _dot_points.push_back(Eigen::Vector2f(x, y));
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_dot_test");
  ros::NodeHandle nh;

  ros::Publisher pub_map_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1);
  ros::Publisher pub_dot_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/dot_cloud", 1);

  std::string map_path = "/home/hkw/global_cloud.pcd";
  std::string dot_point_path = "/home/hkw/Point_List.yaml";

  std::vector<Eigen::Vector2f> dot_points;
  LoadPoints(dot_point_path, dot_points);

  CloudTypePtr map_cloud(new CloudType);
  pcl::io::loadPCDFile(map_path, *map_cloud);

  CloudTypePtr dot_cloud(new CloudType);
  for (size_t i = 0; i < dot_points.size(); i++) {
    PointType p;
    p.x = dot_points[i](0);
    p.y = dot_points[i](1);
    p.z = 0;
    dot_cloud->push_back(p);
  }

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.frame_id = "map";

  sensor_msgs::PointCloud2 dot_msg;
  pcl::toROSMsg(*dot_cloud, dot_msg);
  dot_msg.header.frame_id = "map";

  ros::Rate rate(1);
  while (ros::ok()) {
    pub_map_cloud.publish(map_msg);
    pub_dot_cloud.publish(dot_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}