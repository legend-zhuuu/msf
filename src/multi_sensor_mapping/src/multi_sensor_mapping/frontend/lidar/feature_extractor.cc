#include "multi_sensor_mapping/frontend/lidar/feature_extractor.h"

namespace multi_sensor_mapping {

/**
 * @brief The ByValue struct 根据曲率排序
 */
struct ByValue {
  bool operator()(Smoothness const &left, Smoothness const &right) {
    return left.value < right.value;
  }
};

FeatureExtractor::FeatureExtractor()
    : num_ring_(16),
      horizon_scan_(1800),
      edge_threshold_(0.1),
      surf_threshold_(1.0),
      surf_feature_leaf_size_(0.1) {}

FeatureExtractor::FeatureExtractor(int _num_ring, int _horizon_scan,
                                   double _edge_threshold,
                                   double _surf_threshold,
                                   double _surf_feature_leaf_size)
    : num_ring_(_num_ring),
      horizon_scan_(_horizon_scan),
      edge_threshold_(_edge_threshold),
      surf_threshold_(_surf_threshold),
      surf_feature_leaf_size_(_surf_feature_leaf_size) {
  AllocateMemory();
}

FeatureExtractor::~FeatureExtractor() {
  delete[] cloud_curvature_;
  delete[] cloud_neighbor_picked_;
  delete[] cloud_label_;
}

void FeatureExtractor::SetParams(int _num_ring, int _horizon_scan,
                                 double _edge_threshold, double _surf_threshold,
                                 double _surf_feature_leaf_size) {
  num_ring_ = _num_ring;
  horizon_scan_ = _horizon_scan;
  edge_threshold_ = _edge_threshold;
  surf_threshold_ = _surf_threshold;
  surf_feature_leaf_size_ = _surf_feature_leaf_size;
  AllocateMemory();
}

void FeatureExtractor::ExtractLOAMFeature(VelRTPointCloudPtr &_full_cloud,
                                          VelRTPointCloudPtr &_corner_cloud,
                                          VelRTPointCloudPtr &_surface_cloud) {
  VelRTPointCloudPtr corresponding_cloud(new VelRTPointCloud);
  ProjectPointCloud(_full_cloud, range_mat_, corresponding_cloud);
  CloudExtraction(corresponding_cloud);
  CaculateSmoothness();
  MarkOccludedPoints();
  ExtractFeatures(_corner_cloud, _surface_cloud);
}

void FeatureExtractor::ExtractLOAMFeature(VelRTPointCloudPtr &_full_cloud,
                                          CloudTypePtr &_corner_cloud,
                                          CloudTypePtr &_surface_cloud) {
  VelRTPointCloudPtr temp_corner(new VelRTPointCloud);
  VelRTPointCloudPtr temp_surface(new VelRTPointCloud);
  ExtractLOAMFeature(_full_cloud, temp_corner, temp_surface);

  CloudConvert(temp_corner, _corner_cloud);
  CloudConvert(temp_surface, _surface_cloud);
}

void FeatureExtractor::AllocateMemory() {
  int point_num = num_ring_ * horizon_scan_;
  point_range_list_.assign(point_num, 0);
  point_column_id_.assign(point_num, 0);
  start_ring_index_.assign(num_ring_, 0);
  end_ring_index_.assign(num_ring_, 0);

  cloud_smoothness_.resize(point_num);

  cloud_curvature_ = new float[point_num];
  cloud_neighbor_picked_ = new int[point_num];
  cloud_label_ = new int[point_num];

  range_mat_ =
      cv::Mat(num_ring_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));
  extracted_cloud_ptr_.reset(new VelRTPointCloud());
  down_size_filter_.SetResolution(surf_feature_leaf_size_);
}

void FeatureExtractor::ProjectPointCloud(
    const VelRTPointCloudPtr &_cloud, cv::Mat &dist_image,
    VelRTPointCloudPtr &corresponding_cloud) {
  corresponding_cloud->resize(num_ring_ * horizon_scan_);
  dist_image =
      cv::Mat(num_ring_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));

  for (const VelRTPoint &p : _cloud->points) {
    if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y) || !pcl_isfinite(p.z))
      continue;
    int row_id = p.ring;
    if (row_id < 0 || row_id >= num_ring_) continue;
    float horizon_angle = atan2(p.x, p.y) * 180.0 / M_PI;

    float angle_resolution = 360.0 / float(horizon_scan_);
    int column_id =
        -round((horizon_angle - 90.0) / angle_resolution) + horizon_scan_ / 2;
    if (column_id >= horizon_scan_) column_id -= horizon_scan_;
    if (column_id < 0 || column_id >= horizon_scan_) continue;

    float range = utils::PointDistance<VelRTPoint>(p);

    if (dist_image.at<float>(row_id, column_id) != FLT_MAX) continue;
    dist_image.at<float>(row_id, column_id) = range;

    int index = column_id + row_id * horizon_scan_;
    corresponding_cloud->points[index] = p;
  }
}

void FeatureExtractor::CloudExtraction(VelRTPointCloudPtr &_cloud) {
  extracted_cloud_ptr_->clear();
  int point_index = 0;
  for (int i = 0; i < num_ring_; i++) {
    start_ring_index_[i] = point_index - 1 + 5;
    for (int j = 0; j < horizon_scan_; j++) {
      if (range_mat_.at<float>(i, j) != FLT_MAX) {
        point_column_id_[point_index] = j;
        point_range_list_[point_index] = range_mat_.at<float>(i, j);
        extracted_cloud_ptr_->push_back(_cloud->points[j + i * horizon_scan_]);
        point_index++;
      }
    }
    end_ring_index_[i] = point_index - 1 - 5;
  }
}

void FeatureExtractor::CaculateSmoothness() {
  for (int i = 5; i < extracted_cloud_ptr_->points.size() - 5; i++) {
    float diff_range = point_range_list_[i - 5] + point_range_list_[i - 4] +
                       point_range_list_[i - 3] + point_range_list_[i - 2] +
                       point_range_list_[i - 1] - point_range_list_[i] * 10 +
                       point_range_list_[i + 1] + point_range_list_[i + 2] +
                       point_range_list_[i + 3] + point_range_list_[i + 4] +
                       point_range_list_[i + 5];
    cloud_curvature_[i] = diff_range * diff_range;
    cloud_neighbor_picked_[i] = 0;
    cloud_label_[i] = 0;
    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].ind = i;
  }
}

void FeatureExtractor::MarkOccludedPoints() {
  for (int i = 5; i < extracted_cloud_ptr_->points.size() - 6; i++) {
    float depth1 = point_range_list_[i];
    float depth2 = point_range_list_[i + 1];
    int column_diff =
        std::abs(int(point_column_id_[i + 1] - point_column_id_[i]));

    if (column_diff < 10) {
      if (depth1 - depth2 > 0.3) {
        cloud_neighbor_picked_[i - 5] = 1;
        cloud_neighbor_picked_[i - 4] = 1;
        cloud_neighbor_picked_[i - 3] = 1;
        cloud_neighbor_picked_[i - 2] = 1;
        cloud_neighbor_picked_[i - 1] = 1;
        cloud_neighbor_picked_[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloud_neighbor_picked_[i + 1] = 1;
        cloud_neighbor_picked_[i + 2] = 1;
        cloud_neighbor_picked_[i + 3] = 1;
        cloud_neighbor_picked_[i + 4] = 1;
        cloud_neighbor_picked_[i + 5] = 1;
        cloud_neighbor_picked_[i + 6] = 1;
      }
    }

    float diff1 =
        std::abs(float(point_range_list_[i - 1] - point_range_list_[i]));
    float diff2 =
        std::abs(float(point_range_list_[i + 1] - point_range_list_[i]));

    if (diff1 > 0.02 * point_range_list_[i] &&
        diff2 > 0.02 * point_range_list_[i])
      cloud_neighbor_picked_[i] = 1;
  }
}

void FeatureExtractor::ExtractFeatures(VelRTPointCloudPtr &_corner_cloud,
                                       VelRTPointCloudPtr &_surface_cloud) {
  _corner_cloud->clear();
  _surface_cloud->clear();

  VelRTPointCloudPtr surface_cloud_scan(new VelRTPointCloud());
  VelRTPointCloudPtr surface_cloud_scan_downsample(new VelRTPointCloud());
  for (int i = 0; i < num_ring_; i++) {
    surface_cloud_scan->clear();

    /// 把一个ring分成6份去计算
    for (int j = 0; j < 6; j++) {
      int sp = (start_ring_index_[i] * (6 - j) + end_ring_index_[i] * j) / 6;
      int ep =
          (start_ring_index_[i] * (5 - j) + end_ring_index_[i] * (j + 1)) / 6 -
          1;
      if (sp >= ep) continue;
      std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep,
                ByValue());

      /// 计算角点
      int largest_picked_num = 0;
      for (int k = ep; k >= sp; k--) {
        int index = cloud_smoothness_[k].ind;
        if (cloud_neighbor_picked_[index] == 0 &&
            cloud_curvature_[index] > edge_threshold_) {
          largest_picked_num++;
          if (largest_picked_num <= 20) {
            cloud_label_[index] = 1;
            _corner_cloud->push_back(extracted_cloud_ptr_->points[index]);
          } else {
            break;
          }

          cloud_neighbor_picked_[index] = 1;
          for (int l = 1; l <= 5; l++) {
            int column_diff = std::abs(int(point_column_id_[index + l] -
                                           point_column_id_[index + l - 1]));
            if (column_diff > 10) break;
            cloud_neighbor_picked_[index + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int column_diff = std::abs(int(point_column_id_[index + l] -
                                           point_column_id_[index + l + 1]));
            if (column_diff > 10) break;
            cloud_neighbor_picked_[index + l] = 1;
          }
        }
      }

      /// 计算平面点
      for (int k = sp; k <= ep; k++) {
        int index = cloud_smoothness_[k].ind;
        if (cloud_neighbor_picked_[index] == 0 &&
            cloud_curvature_[index] < surf_threshold_) {
          cloud_label_[index] = -1;
          cloud_neighbor_picked_[index] = 1;

          for (int l = 1; l <= 5; l++) {
            int column_diff = std::abs(int(point_column_id_[index + l] -
                                           point_column_id_[index + l - 1]));
            if (column_diff > 10) break;
            cloud_neighbor_picked_[index + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int column_diff = std::abs(int(point_column_id_[index + l] -
                                           point_column_id_[index + l + 1]));
            if (column_diff > 10) break;
            cloud_neighbor_picked_[index + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloud_label_[k] <= 0) {
          surface_cloud_scan->push_back(extracted_cloud_ptr_->points[k]);
        }
      }
    }

    surface_cloud_scan_downsample->clear();
    down_size_filter_.SetInputCloud(surface_cloud_scan);
    down_size_filter_.Filter(surface_cloud_scan_downsample);
    *_surface_cloud += *surface_cloud_scan_downsample;
  }
}

void FeatureExtractor::CloudConvert(VelRTPointCloudPtr &input,
                                    CloudTypePtr &output) {
  output.reset(new CloudType());
  pcl::copyPointCloud(*input, *output);
}

}  // namespace multi_sensor_mapping
