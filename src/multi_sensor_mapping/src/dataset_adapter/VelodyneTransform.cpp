#include <dataset_adapter/VelodyneTransform.h>
#include <glog/logging.h>

namespace dataset_adapter {

VelodyneTransform::VelodyneTransform(std::string lidar_model,
                                     std::string lidar_frame,
                                     std::string calibration_path,
                                     double min_range, double max_range,
                                     double view_direction, double view_width)
    : data_(new VelodyneRawData), lidar_frame_(lidar_frame) {
  data_->SetParameters(lidar_model, min_range, max_range, view_direction,
                       view_width);
  data_->LoadCalibration(calibration_path);
}

void VelodyneTransform::ProcessScan(
    const velodyne_msgs::VelodyneScan::ConstPtr &scan_msg,
    sensor_msgs::PointCloud2::Ptr out_msg) {
  VelRTPointCloudPtr cloud(new VelRTPointCloud);
  for (size_t i = 0; i < scan_msg->packets.size(); ++i) {
    data_->Unpack(scan_msg->packets[i], cloud, scan_msg->header.stamp);
  }

  pcl::toROSMsg(*cloud, *out_msg);
  out_msg->header.stamp = scan_msg->header.stamp;
  out_msg->header.frame_id = lidar_frame_;
}

}  // namespace dataset_adapter
