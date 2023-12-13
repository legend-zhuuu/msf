#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <mutex>

#include "config.h"
#include "msm_sdk/msm_sdk.h"

using namespace msm_sdk;

ros::Publisher pub_map;

bool image_init_flag = false;
cv::Mat show_img;

std::mutex location_pose_mutex;
bool update_location_pose_flag = false;
Eigen::Vector3f location_pose_pix;

void PutVisualizationImg(const cv::Mat& _img) {
  // sensor_msgs::ImagePtr img_msg =
  //     cv_bridge::CvImage(std_msgs::Header(), "mono8", _img).toImageMsg();
  // pub_map.publish(*img_msg);
  _img.copyTo(show_img);

  image_init_flag = true;
}

void PutLocationPose(const StampedPose& _pose) {
  /// TODO
}

void PutLocation2dPose(const Eigen::Vector3f& _pose_2d) {
  std::cout << "pose 2D : " << _pose_2d(0) << " " << _pose_2d(1) << " "
            << _pose_2d(2) << std::endl;
}

void PutLocationPixPose(const Eigen::Vector3f& _pose_pix) {
  std::lock_guard<std::mutex> lg(location_pose_mutex);
  std::cout << "pose pix : " << _pose_pix(0) << " " << _pose_pix(1) << " "
            << _pose_pix(2) << std::endl;
  location_pose_pix = _pose_pix;
  update_location_pose_flag = true;
}

void PutMarkPointsPixelPose(const std::vector<Eigen::Vector2f>& _pose_vec) {
  /// TODO
}

void PutHighlightMarkPointPixelPose(const Eigen::Vector2f& _pose) {
  /// TODO
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_low_cost_location");
  ros::NodeHandle nh;

  auto msm_version = msm_sdk::GetVersion();
  std::cout << "MSM SDK Version : " << msm_version.toString() << std::endl;

  std::string source_dir = std::string(MSM_SOURCE_DIR);
  std::string param_set_path = source_dir + "/config/mapping_backpack_v4";
  std::string map_path =
      "/home/hkw/workspace/202_mapping_ws/src/backpack-studio/resources/map";

  APILowCostLocation locator;
  locator.SetMapPath(map_path);
  locator.Init(param_set_path);

  // 注册回调函数
  locator.RegVisualizationImgCallback(
      std::bind(PutVisualizationImg, std::placeholders::_1));
  locator.RegLocationPoseCallback(
      std::bind(PutLocationPose, std::placeholders::_1));
  locator.RegLocation2dPoseCallback(
      std::bind(PutLocation2dPose, std::placeholders::_1));
  locator.RegLocationPixPoseCallback(
      std::bind(PutLocationPixPose, std::placeholders::_1));
  locator.RegMarkPointsPixPoseCallback(
      std::bind(PutMarkPointsPixelPose, std::placeholders::_1));
  locator.RegHighlightMarkPointPixPoseCallback(
      std::bind(PutHighlightMarkPointPixelPose, std::placeholders::_1));

  locator.Start();

  std::cout << "map resolution : " << locator.GetMapResolution() << std::endl;

  int cnt = 0;
  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (image_init_flag && update_location_pose_flag) {
      int pose_pix_x, pose_pix_y;
      location_pose_mutex.lock();
      pose_pix_x = location_pose_pix(0);
      pose_pix_y = location_pose_pix(1);
      location_pose_mutex.unlock();
      cv::Mat color_image;
      cv::cvtColor(show_img, color_image, cv::COLOR_GRAY2RGB);
      cv::circle(color_image, cv::Point(pose_pix_y, pose_pix_x), 5,
                 cv::Scalar(255, 0, 0), cv::FILLED);
      cv::imshow("visualization", color_image);
      cv::waitKey(1);
      update_location_pose_flag = false;
      cnt++;
      if (cnt > 200) {
        break;
      }
    }
  }

  locator.Stop();
  std::cout << "mapper stop 1" << std::endl;
  sleep(5);
  locator.SetMapPath(map_path);
  locator.Init(param_set_path);
  sleep(3);
  locator.Start();

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (image_init_flag && update_location_pose_flag) {
      int pose_pix_x, pose_pix_y;
      location_pose_mutex.lock();
      pose_pix_x = location_pose_pix(0);
      pose_pix_y = location_pose_pix(1);
      location_pose_mutex.unlock();
      cv::Mat color_image;
      cv::cvtColor(show_img, color_image, cv::COLOR_GRAY2RGB);
      cv::circle(color_image, cv::Point(pose_pix_y, pose_pix_x), 5,
                 cv::Scalar(255, 0, 0), cv::FILLED);
      cv::imshow("visualization", color_image);
      cv::waitKey(1);
      update_location_pose_flag = false;
    }
  }

  std::cout << "mapper stop " << std::endl;
  ros::spin();

  return 0;
}