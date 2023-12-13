#ifndef LQC_ROS_SENSOR_H_
#define LQC_ROS_SENSOR_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <lqc/lqc.h>

namespace lqc {
namespace _ros {

class CmdVelSubscriber : public lqc::CmdVelSensor {
 public:
  explicit CmdVelSubscriber(const ros::NodeHandle &nh);
  CmdVelSubscriber(const ros::NodeHandle &nh, const YAML::Node &cfg);
  bool init(std::set<Observation> &requirements) override;
  bool update(const std::map<RoboState, ArrX> &state_map,
              const Gamepad::State &gp_state,
              std::map<Observation, ArrX> &ob_map) override;
 private:
  void callback(const geometry_msgs::Twist::ConstPtr &msg);
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Time cmd_vel_stamp_;
  geometry_msgs::Twist cmd_vel_msg_;
  bool enabled_{false};
};

class HeightmapSensor : public lqc::Sensor {
 public:
  explicit HeightmapSensor(const ros::NodeHandle &nh, const std::string &path);
  void sense(std::set<Observation> &result) override;
  bool init(std::set<Observation> &requirements) override;
  bool update(const std::map<RoboState, ArrX> &state_map,
              const Gamepad::State &gp_state,
              std::map<Observation, ArrX> &ob_map) override;

 private:
  using PoseMsgType = geometry_msgs::PoseWithCovarianceStamped;
  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);
  void poseCallback(const PoseMsgType::ConstPtr &msg);
  bool sampleFromGridMap(float x, float y, float &z);

  bool isMapUptodate(const std::string &info) const;
  bool isPoseUptodate(const std::string &info) const;

  ros::NodeHandle nh_;
  std::string topic_name_, layer_name_;

  bool enabled_{false};
  std::mutex msg_mtx_;
  ros::Subscriber grid_map_sub_;
  grid_map::GridMap grid_map_;
  grid_map_msgs::GridMap grid_map_msg_;
  ArrX data_;

  std::string pose_topic_name_;
  ros::Subscriber pose_sub_;
  PoseMsgType pose_msg_;
  Quaternion orn_;

  ros::Publisher sample_pub_;
  bool sample_pub_inited_{false};
  sensor_msgs::PointCloud2 points_;

  union {
    int scan_dots_[2]{11, 11};
    struct { int scan_dots_x_, scan_dots_y_; };
  };
  union {
    float scan_grid_[2]{0.1, 0.1};
    struct { float scan_grid_x_, scan_grid_y_; };
  };
  std::vector<float> x_grids_, y_grids_;
  grid_map::InterpolationMethods interp_;
};

}  // namespace _ros
}  // namespace lqc

#endif  // LQC_ROS_SENSOR_H_
