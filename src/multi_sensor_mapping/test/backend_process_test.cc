#include <ros/ros.h>

#include "config.h"
#include "msm_sdk/msm_sdk.h"

using namespace msm_sdk;

void PutGraph(const Graph& _graph) {
  std::cout << "Graph vertic size : " << _graph.vertices.size()
            << " loop clousre edge size : " << _graph.loop_closure_edges.size()
            << std::endl;
}

void PutGlobalCloud(const StampedCloud& _cloud) {
  std::cout << "Global Cloud size : " << _cloud.cloud.size() << std::endl;
}

void PutLoopClosure(const LoopClosureInfo& _loop_closure) {
  std::cout << "Get Loop Closure  : " << _loop_closure.loop_closure_id
            << std::endl;
}

void PutProcessState(const bool& _process_state) {
  std::cout << "Get Process state : " << _process_state << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_backend_process");
  ros::NodeHandle nh;

  auto msm_version = msm_sdk::GetVersion();
  std::cout << "MSM SDK Version : " << msm_version.toString() << std::endl;

  std::string session_path =
      "/home/hkw/DATA/Dataset/MappingBackpackV3/玉泉校区/"
      "ZjuYuquan(20230403)-Refined-20230404";

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/mapping_backpack_v3";

  APIBackendProcess backend;
  // 初始化
  backend.Init(param_set_path);

  // 注册回调函数
  backend.RegGraphCallback(std::bind(PutGraph, std::placeholders::_1));
  backend.RegGlobalMapCallback(
      std::bind(PutGlobalCloud, std::placeholders::_1));
  backend.RegLoopClosureCallback(
      std::bind(PutLoopClosure, std::placeholders::_1));
  backend.RegProcessStateCallback(
      std::bind(PutProcessState, std::placeholders::_1));
  backend.Start(session_path);

  sleep(10);

  backend.SaveLidarMapSession("");
  // backend.StartAutoLoopClosureDetection();
  // sleep(30);
  // std::cout << "start manual loop clousur" << std::endl;
  // std::getchar();
  // backend.StartManualLoopClosureDetection(1);

  ros::spin();
  backend.Stop();
  return 0;
}