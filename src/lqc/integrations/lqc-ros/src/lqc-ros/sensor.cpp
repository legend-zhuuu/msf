#include <lqc-ros/sensor.h>

namespace lqc {
namespace _ros {

CmdVelSubscriber::CmdVelSubscriber(const ros::NodeHandle &nh) : nh_(nh) {
  std::string topic_name;
  nh_.param<std::string>("LQC_CMD_VEL_TOPIC", topic_name, "cmd_vel");
  cmd_vel_sub_ = nh_.subscribe(topic_name, 1, &CmdVelSubscriber::callback,
                               this, ros::TransportHints().tcpNoDelay());
}

CmdVelSubscriber::CmdVelSubscriber(
    const ros::NodeHandle &nh, const YAML::Node &cfg) : lqc::CmdVelSensor(cfg), nh_(nh) {
  std::string topic_name;
  nh_.param<std::string>("LQC_CMD_VEL_TOPIC", topic_name, "cmd_vel");
  cmd_vel_sub_ = nh_.subscribe(topic_name, 1, &CmdVelSubscriber::callback,
                               this, ros::TransportHints().tcpNoDelay());
}

void CmdVelSubscriber::callback(const geometry_msgs::Twist::ConstPtr &msg) {
  cmd_vel_msg_ = *msg;
  cmd_vel_stamp_ = ros::Time::now();
}

bool CmdVelSubscriber::init(std::set<Observation> &requirements) {
  enabled_ = false;
  return CmdVelSensor::init(requirements);
}

bool CmdVelSubscriber::update(
    const std::map<RoboState, ArrX> &state_map,
    const Gamepad::State &gp_state,
    std::map<Observation, ArrX> &ob_map) {
  if (gp_state.LB.pressed && gp_state.A) {
    enabled_ = !enabled_;
    log("CmdVelSubscriber ", enabled_ ? "enabled." : "disabled.");
  }

  if (!enabled_ or (ros::Time::now() - cmd_vel_stamp_).toSec() > 0.1) {
    return CmdVelSensor::update(state_map, gp_state, ob_map);
  }

  cmd_vel_ << float(cmd_vel_msg_.linear.x),
      float(cmd_vel_msg_.linear.y),
      float(cmd_vel_msg_.angular.z);
  if (cmd_vel_.abs().maxCoeff() < 0.1) {
    cmd_vel_.setZero();
    cmd_stand_.setOnes();
  } else {
    cmd_stand_.setZero();
  }
  ob_map[ob::CmdVel] = cmd_vel_;
  ob_map[ob::CmdStand] = cmd_stand_;
  return true;
}

HeightmapSensor::HeightmapSensor(const ros::NodeHandle &nh, const std::string &path)
    : nh_(nh) {
  auto cfg = YAML::LoadFile(path + "/policy.yml");
  auto node = cfg["observation_options"];
  yaml::assertNTuple(node, "scan_dots", 2);
  yaml::assertNTuple(node, "scan_grid", 2);
  yaml::setTo<int, 2>(node, "scan_dots", scan_dots_);
  yaml::setTo<float, 2>(node, "scan_grid", scan_grid_);
  ob::kSizeMap[ob::HeightScan] = scan_dots_x_ * scan_dots_y_;
  x_grids_.clear();
  y_grids_.clear();
  float x_grid0 = -(scan_dots_x_ - 1) / 2.f * scan_grid_x_;
  for (int i = 0; i < scan_dots_x_; ++i) {
    x_grids_.push_back(x_grid0 + i * scan_grid_x_);
  }
  float y_grid0 = -(scan_dots_y_ - 1) / 2.f * scan_grid_y_;
  for (int i = 0; i < scan_dots_y_; ++i) {
    y_grids_.push_back(y_grid0 + i * scan_grid_y_);
  }
  data_.setZero(scan_dots_x_ * scan_dots_y_);

  log("ros::HeightmapSensor started. Dots [", scan_dots_[0], "x", scan_dots_[1],
      "], grid [", scan_grid_[0], ", ", scan_grid_[1], "].");
  nh_.getParam("LQC_GRIDMAP_TOPIC", topic_name_);
  nh_.getParam("LQC_GRIDMAP_LAYER", layer_name_);
  nh_.getParam("LQC_ODOM_TOPIC", pose_topic_name_);
  grid_map_sub_ = nh_.subscribe(topic_name_, 1, &HeightmapSensor::gridMapCallback,
                                this, ros::TransportHints().tcpNoDelay());
  pose_sub_ = nh_.subscribe(pose_topic_name_, 1, &HeightmapSensor::poseCallback,
                            this, ros::TransportHints().tcpNoDelay());
  interp_ = grid_map::InterpolationMethods::INTER_CUBIC;
}

void HeightmapSensor::sense(std::set<Observation> &result) {
  result.insert(ob::HeightScan);
}

bool HeightmapSensor::init(std::set<Observation> &requirements) {
  enabled_ = false;
  return true;
}

bool HeightmapSensor::isMapUptodate(const std::string &info) const {
  auto heightmap_lag = ros::Time::now() - grid_map_msg_.info.header.stamp;
  if (heightmap_lag < ros::Duration(0.8)) return true;
  lg::warn(info, " due to outdated heightmap received ", heightmap_lag, "s ago.");
  return false;
}

bool HeightmapSensor::isPoseUptodate(const std::string &info) const {
  auto pose_lag = ros::Time::now() - pose_msg_.header.stamp;
  if (pose_lag < ros::Duration(0.5)) return true;
  lg::warn(info, " due to outdated pose received ", pose_lag, "s ago.");
  return false;
}

bool HeightmapSensor::update(
    const std::map<RoboState, ArrX> &state_map,
    const Gamepad::State &gp_state,
    std::map<Observation, ArrX> &ob_map) {
  data_.setZero();
  if (gp_state.LB.pressed && gp_state.B) {
    enabled_ = !enabled_;
    log("Heightmap ", enabled_ ? "enabled." : "disabled.");
  }

  if (enabled_) {
    if (!isMapUptodate("Heightmap disabled") or
        !isPoseUptodate("Heightmap disabled")) {
      enabled_ = false;
    } else {
      const auto &layers = grid_map_msg_.layers;
      if (std::find(layers.begin(), layers.end(), layer_name_) == layers.end()) {
        lg::crit("No gridmap layer named ", layer_name_, ". Heightmap disabled.");
        enabled_ = false;
      }
    }
  }

  if (!enabled_) {
    ob_map[ob::HeightScan] = data_;
    return true;
  }

  LockGuard lock(msg_mtx_);
  const auto &data = grid_map_[layer_name_];
  if (data.numberOfFinites() < 0.8 * data.size()) {
    ob_map[ob::HeightScan] = data_;
    return true;
  }

  float z0;
  const auto &pos = pose_msg_.pose.pose.position;
  const auto &map_pos = grid_map_msg_.info.pose.position;
  if (!sampleFromGridMap(pos.x, pos.y, z0)) {
    ob_map[ob::HeightScan] = data_;
    return true;
  }

  float yaw = orn_.rpy()[2];
  float cy = std::cos(yaw), sy = std::sin(yaw);
  float x, y, z;
  int idx = 0;
  float points[3 * scan_dots_x_ * scan_dots_y_];
  for (auto x_grid : x_grids_) {
    for (auto y_grid : y_grids_) {
      /// convert to world frame
      x = pos.x + cy * x_grid - sy * y_grid;
      y = pos.y + sy * x_grid + cy * y_grid;
      if (sampleFromGridMap(x, y, z)) {
        z = clip(z - z0, -1.f, 1.f);
      } else {
        z = 0.;
      }

      data_[idx] = z;
      points[idx * 3] = x;
      points[idx * 3 + 1] = y;
      points[idx * 3 + 2] = z + z0 + map_pos.z;
      idx++;
    }
  }

  ob_map[ob::HeightScan] = data_;

  if (logger_) {
    if (!sample_pub_inited_) {
      sample_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("height_scan", 1);
      sensor_msgs::PointCloud2Modifier modifier(points_);
      modifier.setPointCloud2Fields(
          3,
          "x", 1, sensor_msgs::PointField::FLOAT32,
          "y", 1, sensor_msgs::PointField::FLOAT32,
          "z", 1, sensor_msgs::PointField::FLOAT32);
      points_.width = scan_dots_y_;
      points_.height = scan_dots_x_;
      points_.point_step = 3 * sizeof(float);
      points_.row_step = points_.point_step * points_.width;
      points_.data.resize(points_.row_step * points_.height);
      points_.is_dense = false;
      points_.is_bigendian = false;
      sample_pub_inited_ = true;
    }
    points_.header = pose_msg_.header;
    std::memcpy(points_.data.data(), points, points_.data.size());
    sample_pub_.publish(points_);
  }

  return true;
}

void HeightmapSensor::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg) {
  LockGuard lock(msg_mtx_);
  grid_map_msg_ = *msg;
  if (msg->layers.empty()) return;
  grid_map::GridMapRosConverter::fromMessage(
      *msg, grid_map_, {layer_name_}, false, false);
}

void HeightmapSensor::poseCallback(const PoseMsgType::ConstPtr &msg) {
  LockGuard lock(msg_mtx_);
  pose_msg_ = *msg;
//  std::cout<<pose_msg_.header.stamp<<std::endl;
  orn_.set(msg->pose.pose.orientation.w,
           msg->pose.pose.orientation.x,
           msg->pose.pose.orientation.y,
           msg->pose.pose.orientation.z);
}

bool HeightmapSensor::sampleFromGridMap(float x, float y, float &z) {
  try {
    z = grid_map_.atPosition(layer_name_, {x, y}, interp_);
  } catch (const std::out_of_range &) {
    auto center = grid_map_.getPosition();
    log("[", x, ", ", y, "] out of map centered [",
        center.x(), ", ", center.y(), "]!");
    return false;
  }
  if (std::isinf(z) or std::isnan(z)) return false;
  return true;
}

}  // namespace _ros
}  // namespace lqc