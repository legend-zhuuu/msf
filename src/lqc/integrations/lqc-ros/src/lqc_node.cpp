#include <lqc-ros/lqc-ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lqc_node");
  ros::NodeHandle nh;
  lqc::Agent agent;

  std::string policy_dir;
  if (nh.hasParam("LQC_POLICY_DIR")) {  // for roslaunch
    nh.getParam("LQC_POLICY_DIR", policy_dir);
  } else if (argc != 2) {
    lqc::lg::crit("Param `LQC_POLICY_DIR` not specified!");
    return -1;
  } else {  // for rosrun
    policy_dir = argv[1];
  }

  agent.logger<lqc::ros::Logger>(nh);
  if (!std::ifstream(policy_dir + "/policy.yml").good()) {
    agent.policy<lqc::LegacyPolicy>(policy_dir);
  } else {
    auto policy = std::make_shared<lqc::PolicyV2>(policy_dir);
    agent.policy(policy);
    if (std::ifstream(policy_dir + "/gamepad.yml").good()) {
      auto gamepad_cfg = YAML::LoadFile(policy_dir + "/gamepad.yml");
      policy->sensor<lqc::ros::CmdVelSubscriber>(nh, gamepad_cfg);
    } else {
      policy->sensor<lqc::ros::CmdVelSubscriber>(nh);
    }
    if (policy->requires(lqc::Observation::HeightScan)) {
      policy->sensor<lqc::ros::HeightmapSensor>(nh, policy_dir);
    }
  }
  agent.start();
  ros::spin();
  return 0;
}
