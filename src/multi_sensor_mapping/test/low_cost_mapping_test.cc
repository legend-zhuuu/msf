#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include "config.h"
#include "msm_sdk/msm_sdk.h"

using namespace msm_sdk;

ros::Publisher pub_map;

void PutVisualizationImg(const cv::Mat& _img) {
  sensor_msgs::ImagePtr img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", _img).toImageMsg();
  pub_map.publish(*img_msg);
}

void PutProgress(const double& _progress) {
  std::cout << "progress : " << _progress << std::endl;
}

void PutKeyFramePixelPoseCallback(
    const std::vector<Eigen::Vector2f>& _key_frame_poses) {
  std::cout << "key frame size : " << _key_frame_poses.size() << std::endl;
}

void PutTagPixelPoseCallback(const Eigen::Vector3f& _tag_pixel_pose) {
  std::cout << "tag pixel pose : " << _tag_pixel_pose(0) << " : "
            << _tag_pixel_pose(1) << std::endl;
}

void PutSensorAbnormalCallback(const bool& _sensor_abnormal) {
  std::cout << "sensor abnormal : " << _sensor_abnormal << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_low_cost_mapping");
  ros::NodeHandle nh;

  pub_map = nh.advertise<sensor_msgs::Image>("image", 10);

  auto msm_version = msm_sdk::GetVersion();
  std::cout << "MSM SDK Version : " << msm_version.toString() << std::endl;

  std::string log_path = "/home/hkw/mapping.log";
  msm_sdk::SetLogPath(log_path);

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/mapping_backpack_v4";

  std::string cache_path = "/home/hkw/shihushan";

  APILowCostMapping mapper;
  // 初始化
  mapper.Init(param_set_path);
  mapper.SetMapCachePath(cache_path);

  // 注册回调函数
  mapper.RegVisualizationImgCallback(
      std::bind(PutVisualizationImg, std::placeholders::_1));
  mapper.RegProgressCallback(std::bind(PutProgress, std::placeholders::_1));
  mapper.RegKeyFramePixelPoseCallback(
      std::bind(PutKeyFramePixelPoseCallback, std::placeholders::_1));
  mapper.RegTagPixelPoseCallback(
      std::bind(PutTagPixelPoseCallback, std::placeholders::_1));
  mapper.RegSensorAbnormalCallback(
      std::bind(PutSensorAbnormalCallback, std::placeholders::_1));

  mapper.Start();

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "Type in < a > for start backend, < b > for stop "
              << std::endl;

    char con = std::getchar();
    if (con == 'a') {
      std::cout << "External command : Start Backend" << std::endl;
      mapper.StartBackend();
    } else if (con == 'b') {
      std::cout << "External command : Stop" << std::endl;
      mapper.Stop();
    } else if (con == 'c') {
      std::cout << "External command : Start" << std::endl;
      mapper.Start();
    } else if (con == 'd') {
      std::cout << "External command : Break" << std::endl;
      break;
    }
  }
  mapper.Stop();

  std::cout << "mapper stop " << std::endl;
  ros::spin();

  return 0;
}