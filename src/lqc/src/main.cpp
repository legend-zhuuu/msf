#include <lqc/lqc.h>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    lqc::log("Usage: lqc_node <policy_dir>");
    return -1;
  }

  lqc::Agent agent;
  std::string policy_dir{argv[1]};
  if (!std::ifstream(policy_dir + "/policy.yml").good()) {
    agent.policy<lqc::LegacyPolicy>(policy_dir);
  } else {
    auto policy = std::make_shared<lqc::PolicyV2>(policy_dir);
    if (std::ifstream(policy_dir + "/gamepad.yml").good()) {
      auto gamepad_cfg = YAML::LoadFile(policy_dir + "/gamepad.yml");
      policy->sensor<lqc::CmdVelSensor>(gamepad_cfg);
    } else {
      policy->sensor<lqc::CmdVelSensor>();
    }
    agent.policy(policy);
  }

  agent.start();
  while (true);
  return 0;
}
