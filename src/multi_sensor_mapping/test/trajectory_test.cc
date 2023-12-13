#include "multi_sensor_mapping/spline/trajectory.h"

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "config.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/spline/trajectory_estimator.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

using namespace multi_sensor_mapping;

std::vector<PoseData2> LoadKaistGroundTruth(std::string _gt_path) {
  std::vector<PoseData2> gt_pose_data_vec;

  std::ifstream infile;
  infile.open(_gt_path);
  std::string current_line;
  std::vector<Eigen::VectorXd> info;
  int i = 0;
  while (std::getline(infile, current_line)) {
    std::istringstream s(current_line);
    std::string field;
    std::vector<double> vec;
    while (getline(s, field, ',')) {
      if (field.empty()) continue;
      vec.push_back(atof(field.c_str()));
    }
    Eigen::VectorXd temp(vec.size());
    for (size_t i = 0; i < vec.size(); i++) {
      temp(i) = vec.at(i);
    }

    info.push_back(temp);
  }

  Eigen::Vector3d first_trans, pose_trans;
  int64_t first_timestamp = 0;
  for (size_t i = 0; i < info.size(); i++) {
    Eigen::VectorXd temp = info.at(i);

    int64_t timestamp = int64_t(temp[0]);
    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    pose_matrix(0, 0) = temp[1];
    pose_matrix(0, 1) = temp[2];
    pose_matrix(0, 2) = temp[3];
    pose_matrix(0, 3) = temp[4];
    pose_matrix(1, 0) = temp[5];
    pose_matrix(1, 1) = temp[6];
    pose_matrix(1, 2) = temp[7];
    pose_matrix(1, 3) = temp[8];
    pose_matrix(2, 0) = temp[9];
    pose_matrix(2, 1) = temp[10];
    pose_matrix(2, 2) = temp[11];
    pose_matrix(2, 3) = temp[12];

    if (i == 0) {
      first_trans = pose_matrix.block<3, 1>(0, 3);
      first_timestamp = timestamp;
    }

    pose_trans = (pose_matrix.block<3, 1>(0, 3) - first_trans);
    Eigen::Quaterniond q(pose_matrix.block<3, 3>(0, 0));

    PoseData2 pose_data;
    pose_data.timestamp_ns = timestamp - first_timestamp;
    pose_data.position = pose_trans;
    pose_data.orientation = SO3d(q);
    gt_pose_data_vec.push_back(pose_data);
  }

  std::cout << "Load gt pose size : " << gt_pose_data_vec.size() << std::endl;
  return gt_pose_data_vec;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_test");
  ros::NodeHandle nh;

  ros::Publisher pub_trajectory = nh.advertise<nav_msgs::Path>("trajectory", 1);

  std::string gt_path =
      "/home/hkw/DATA/Dataset/KAIST/campus00/campus00_pose/campus00/"
      "global_pose.csv";

  std::string sensor_param_path =
      std::string(MSM_SOURCE_DIR) + "/config/template/sensor_param.yaml";
  std::string extrinsic_param_path =
      std::string(MSM_SOURCE_DIR) + "/config/template/extrinsic_param.yaml";

  SensorParams::Ptr sensor_params(new SensorParams);
  sensor_params->Load(sensor_param_path);

  ExtrinsicParams::Ptr extrinsic_params(new ExtrinsicParams);
  extrinsic_params->Load(extrinsic_param_path);

  std::vector<PoseData2> gt_pose_data_vec = LoadKaistGroundTruth(gt_path);

  double knot_distance = 0.1;
  std::shared_ptr<Trajectory> trajectory(new Trajectory(knot_distance));
  trajectory->SetExtrinsicParams(sensor_params, extrinsic_params);

  // 初始化轨迹
  Eigen::Vector3d init_pos_knot(Eigen::Vector3d::Zero());
  SO3d init_rot_knot(Eigen::Quaterniond::Identity());
  trajectory->extendKnotsTo(gt_pose_data_vec.back().timestamp_ns, init_rot_knot,
                            init_pos_knot);

  // 轨迹拟合
  TrajectoryEstimatorOptions estimator_option;
  TrajectoryEstimator trajectory_estimator(trajectory, estimator_option);

  Eigen::Matrix<double, 6, 1> info_vec;
  info_vec << 1, 1, 1, 1, 1, 1;
  for (size_t i = 0; i < gt_pose_data_vec.size(); i++) {
    trajectory_estimator.AddIMUPoseMeasurementAnalytic(gt_pose_data_vec.at(i),
                                                       info_vec);
  }
  trajectory_estimator.Solve(100, true);

  // 轨迹可视化
  nav_msgs::Path trajectory_path;
  int64_t min_time = trajectory->MinTimeNs(IMU) + 1;
  int64_t max_time = trajectory->MaxTimeNs(IMU) - 1;
  int64_t dt = 0.1 * 1e9;

  for (int64_t t = min_time; t < max_time; t += dt) {
    SE3d pose = trajectory->GetIMUPose(t);

    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x = pose.translation()(0);
    path_point.pose.position.y = pose.translation()(1);
    path_point.pose.position.z = pose.translation()(2);
    path_point.pose.orientation.w = pose.unit_quaternion().w();
    path_point.pose.orientation.x = pose.unit_quaternion().x();
    path_point.pose.orientation.y = pose.unit_quaternion().y();
    path_point.pose.orientation.z = pose.unit_quaternion().z();

    trajectory_path.poses.push_back(path_point);
  }
  trajectory_path.header.frame_id = "map";

  ros::Rate rate(1);
  while (ros::ok()) {
    pub_trajectory.publish(trajectory_path);
    rate.sleep();
  }

  return 0;
}