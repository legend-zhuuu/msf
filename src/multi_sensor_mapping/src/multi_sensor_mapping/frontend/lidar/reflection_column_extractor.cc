#include "multi_sensor_mapping/frontend/lidar/reflection_column_extractor.h"

namespace multi_sensor_mapping {

static bool cmp_angle(const PointType& p_a, const PointType& p_b) {
  float angle_a = acos(p_a.x / sqrt(p_a.x * p_a.x + p_a.y * p_a.y));
  float angle_b = acos(p_b.x / sqrt(p_b.x * p_b.x + p_b.y * p_b.y));

  return angle_a < angle_b;
}

static bool cmp_intensity(const PointType& p_a, const PointType& p_b) {
  return p_a.intensity > p_b.intensity;
}

ReflectionColumnExtractor::ReflectionColumnExtractor()
    : interest_radius_(5),
      column_diamater_(0.05),
      interest_cloud_(new CloudType),
      high_intensity_cloud_(new CloudType) {}

ReflectionColumnExtractor::ReflectionColumnExtractor(float _interest_radius,
                                                     float _column_diamater)
    : interest_radius_(_interest_radius),
      column_diamater_(_column_diamater),
      interest_cloud_(new CloudType),
      high_intensity_cloud_(new CloudType) {}

std::vector<PointType> ReflectionColumnExtractor::ExtractColumns(
    const CloudTypePtr& _cloud) {
  std::vector<PointType> column_center_vec;
  // 提取ROI点云
  GetInterestPointCloud(_cloud);

  kdtree_cloud_.setInputCloud(interest_cloud_);

  if (high_intensity_cloud_->empty()) {
    return column_center_vec;
  }

  // 欧式聚类
  pcl::search::KdTree<PointType>::Ptr tree_raw_cloud(
      new pcl::search::KdTree<PointType>);
  tree_raw_cloud->setInputCloud(high_intensity_cloud_);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(0.2);  // 不同线束之间距离较大
  ec.setMinClusterSize(5);      // 最小聚类数
  ec.setMaxClusterSize(1000);   // 最大聚类数
  ec.setSearchMethod(tree_raw_cloud);
  ec.setInputCloud(high_intensity_cloud_);
  ec.extract(cluster_indices);

  // 提取反光柱中心点
  for (auto cluster : cluster_indices) {
    multi_layer_cloud_.clear();
    CloudType cloud_cluster;
    std::set<int> z_set;
    // 提取聚类内的点云信息
    for (const auto& idx : cluster.indices) {
      // 直接在 _interest_cloud中进行KNN 扩充点云并分层
      int z_100 = (int)(high_intensity_cloud_->points[idx].z * 100);
      if (z_set.find(z_100) == z_set.end()) {
        z_set.insert(z_100);
        GetMultiLayerCloud(high_intensity_cloud_->points[idx]);
      }

      cloud_cluster.push_back((*high_intensity_cloud_)[idx]);
    }

    PointType column_center = FindCenterOfColumnCluster();
    if (column_center.x && column_center.y) {
      column_center_vec.push_back(column_center);
    }
  }

  return column_center_vec;
}

void ReflectionColumnExtractor::GetInterestPointCloud(
    const CloudTypePtr& _cloud_in) {
  high_intensity_cloud_->clear();
  interest_cloud_->clear();

  for (auto p : _cloud_in->points) {
    // remove nan
    if (pcl_isnan(p.x) || pcl_isnan(p.y) || pcl_isnan(p.z)) {
      continue;
    }

    float point_ds = PointDistanceXY(p);
    if (point_ds > interest_radius_ || p.z > 0.2 || p.z < -0.6) {
      continue;
    }

    interest_cloud_->push_back(p);

    // get high intensity further

    if (p.intensity < 150) {
      // 避免距离过近点云强度过低欧式聚类提取失败的情况
      if (point_ds < interest_radius_ / 5 && p.intensity < 125) {
      } else {
        continue;
      }
    }

    high_intensity_cloud_->push_back(p);
  }
}

void ReflectionColumnExtractor::GetMultiLayerCloud(
    const PointType& _search_point) {
  CloudType single_layer_cloud;

  std::vector<int> k_indices_near;
  std::vector<float> k_sqr_distances_near;
  if (kdtree_cloud_.radiusSearch(_search_point, column_diamater_,
                                 k_indices_near, k_sqr_distances_near)) {
    // 以线束为单位进行判别和处理
    for (size_t k = 0; k < k_indices_near.size(); k++) {
      PointType p = (*interest_cloud_).points[k_indices_near[k]];

      //只处理同一线束的点云
      if (abs(p.z * 100 - _search_point.z * 100) <= 2 && p.intensity >= 100) {
        single_layer_cloud.push_back(p);
      }
    }

    // 存储该线束的点云
    if (single_layer_cloud.size() > 2) {
      multi_layer_cloud_.push_back(single_layer_cloud);
    }
  }
}

PointType ReflectionColumnExtractor::FindCenterOfSingleLayer(
    CloudType& _single_layer_cloud) {
  float max_intensity = 0.0f;
  int cloud_size = _single_layer_cloud.size();

  PointType side_point[2];
  PointType center_point;
  int mid_index = cloud_size / 2;
  if (mid_index < 2) {
    center_point.intensity = 0;
    return center_point;
  }

  // 距离过长 消除误判
  std::sort(_single_layer_cloud.begin(), _single_layer_cloud.end(), cmp_angle);

  float cloud_length = sqrt(
      pow(_single_layer_cloud[0].x - _single_layer_cloud[cloud_size - 1].x, 2) +
      pow(_single_layer_cloud[0].y - _single_layer_cloud[cloud_size - 1].y, 2));
  if (cloud_length > 2 * column_diamater_ ||
      ((_single_layer_cloud[0].intensity +
        _single_layer_cloud[cloud_size - 1].intensity) /
           2 >
       0.9 * (_single_layer_cloud[cloud_size / 2].intensity +
              _single_layer_cloud[cloud_size / 2 + 1].intensity / 2))) {
    center_point.intensity = 0;
    return center_point;
  }

  // oneside
  std::sort(_single_layer_cloud.begin(),
            _single_layer_cloud.begin() + mid_index, cmp_intensity);

  // 确保强度差
  if (_single_layer_cloud[mid_index - 1].intensity >
          0.9 * _single_layer_cloud[0].intensity &&
      _single_layer_cloud[mid_index - 1].intensity < 200) {
    return center_point;
  }
  max_intensity = _single_layer_cloud[0].intensity;

  for (int i = 0; i < mid_index; i++) {
    if (_single_layer_cloud[0].intensity == 255.0f) {
      if ((_single_layer_cloud[i].intensity != 255.0f &&
           _single_layer_cloud[i].intensity < 200.0f) ||
          i == mid_index - 1) {
        side_point[0].x = _single_layer_cloud[i - 1].x;
        side_point[0].y = _single_layer_cloud[i - 1].y;
        if (i < mid_index - 1)
          side_point[0].intensity = _single_layer_cloud[i + 1].intensity;
        else
          side_point[0].intensity = _single_layer_cloud[i].intensity;
        break;
      }
    } else {
      int temp_mid_index = mid_index / 2;
      side_point[0].x = _single_layer_cloud[temp_mid_index - 1].x;
      side_point[0].y = _single_layer_cloud[temp_mid_index - 1].y;
      side_point[0].intensity =
          _single_layer_cloud[temp_mid_index + 1].intensity;
      break;
    }
  }

  // other side
  sort(_single_layer_cloud.begin() + mid_index, _single_layer_cloud.end(),
       cmp_intensity);
  // 确保强度差
  if (_single_layer_cloud[cloud_size - 1].intensity >
          0.9 * _single_layer_cloud[mid_index].intensity &&
      _single_layer_cloud[cloud_size - 1].intensity < 200) {
    return center_point;
  }

  if (_single_layer_cloud[mid_index].intensity > max_intensity) {
    max_intensity = _single_layer_cloud[mid_index].intensity;
  }

  for (int i = mid_index; i < cloud_size; i++) {
    if (_single_layer_cloud[mid_index].intensity == 255.0f) {
      if ((_single_layer_cloud[i].intensity != 255.0f &&
           _single_layer_cloud[i].intensity < 200.0f) ||
          i == cloud_size - 1) {
        side_point[1].x = _single_layer_cloud[i - 1].x;
        side_point[1].y = _single_layer_cloud[i - 1].y;
        if (i < cloud_size - 1)
          side_point[1].intensity = _single_layer_cloud[i + 1].intensity;
        else
          side_point[1].intensity = _single_layer_cloud[i].intensity;
        break;
      }
    } else {
      int temp_mid_index = (mid_index + cloud_size) / 2;
      side_point[1].x = _single_layer_cloud[temp_mid_index - 1].x;
      side_point[1].y = _single_layer_cloud[temp_mid_index - 1].y;
      side_point[1].intensity =
          _single_layer_cloud[temp_mid_index + 1].intensity;
      break;
    }
  }

  // 通过 side points的强度值 分别求解对应的夹角
  double angle[2] = {0};
  for (int i = 0; i < 2; i++) {
    angle[i] = acos(side_point[i].intensity / max_intensity);
  }

  float judge_cos = cos((angle[0] + angle[1]) / 2.0);

  //点云两端的长度除以直径即为sin值
  cloud_length = sqrt(pow(side_point[0].x - side_point[1].x, 2) +
                      pow(side_point[0].y - side_point[1].y, 2));
  float judge_sin = cloud_length / column_diamater_;

  // get the center point of the refelction column
  PointType full_intensity_center, high_intensity_center;
  int full_intensity_num = 0, high_intensity_num = 0;

  // 计算圆表面中心值
  for (int i = 0; i < cloud_size; i++) {
    // 最大强度点云
    if (_single_layer_cloud[i].intensity + 10 >= 255) {
      full_intensity_num++;

      full_intensity_center.x +=
          (_single_layer_cloud[i].x - full_intensity_center.x) /
          full_intensity_num;
      full_intensity_center.y +=
          (_single_layer_cloud[i].y - full_intensity_center.y) /
          full_intensity_num;
    }

    // 高强度点云
    if (_single_layer_cloud[i].intensity > side_point[0].intensity ||
        _single_layer_cloud[i].intensity > side_point[1].intensity) {
      high_intensity_num++;

      high_intensity_center.x +=
          (_single_layer_cloud[i].x - high_intensity_center.x) /
          high_intensity_num;
      high_intensity_center.y +=
          (_single_layer_cloud[i].y - high_intensity_center.y) /
          high_intensity_num;
    }
  }

  // 根据理论推断
  float judge_circle = judge_sin * judge_sin + judge_cos * judge_cos - 1.0f;

  // 倾向选择最大强度的点云作为圆表面中心点
  if (full_intensity_num >= cloud_size / 3) {
    center_point = full_intensity_center;
  } else if (high_intensity_num >= cloud_size / 2) {
    center_point = high_intensity_center;
  } else if (full_intensity_num + high_intensity_num >= cloud_size / 2) {
    center_point.x = (full_intensity_num * full_intensity_center.x +
                      high_intensity_num * high_intensity_center.x) /
                     (full_intensity_num + high_intensity_num);
    center_point.y = (full_intensity_num * full_intensity_center.y +
                      high_intensity_num * high_intensity_center.y) /
                     (full_intensity_num + high_intensity_num);
  } else {
    center_point.x = 0;
    center_point.y = 0;
  }

  if (fabs(judge_circle) < 0.4 || (center_point.x && center_point.y)) {
    float hypotenuse_length =
        sqrt(center_point.x * center_point.x + center_point.y * center_point.y);
    float coefficient_center =
        1.0f + column_diamater_ / 2.0f / hypotenuse_length;

    center_point.x *= coefficient_center;
    center_point.y *= coefficient_center;
  }
  center_point.intensity = 1;

  return center_point;
}

PointType ReflectionColumnExtractor::FindCenterOfColumnCluster() {
  PointType ave_center_point;
  size_t cloud_size = multi_layer_cloud_.size();
  int valid_num = 0;

  // 有效点较少
  if (cloud_size < 2) {
    ave_center_point.x = 0.0f;
    ave_center_point.y = 0.0f;
    return ave_center_point;
  }

  // 分层遍历 取均值
  for (size_t j = 0; j < cloud_size; j++) {
    // 传入单层点云信息
    PointType center_point = FindCenterOfSingleLayer(multi_layer_cloud_[j]);

    if (center_point.x && center_point.y) {
      // 垂直一致性判断
      if (ave_center_point.x && ave_center_point.y) {
        if (j > cloud_size / 3 &&
            (fabs(ave_center_point.x - center_point.x) > column_diamater_ ||
             fabs(ave_center_point.y - center_point.y) > column_diamater_)) {
          continue;
        }
      }

      valid_num++;
      ave_center_point.x += (center_point.x - ave_center_point.x) / valid_num;
      ave_center_point.y += (center_point.y - ave_center_point.y) / valid_num;
    }
  }

  // 有效层较少
  if (valid_num < cloud_size * 2 / 3) {
    ave_center_point.x = 0.0f;
    ave_center_point.y = 0.0f;
  }

  if (PointDistanceXY(ave_center_point) < interest_radius_ / 3) {
    if (valid_num < 5) {
      ave_center_point.x = 0.0f;
      ave_center_point.y = 0.0f;
    }
  } else {
    if (valid_num < 2) {
      ave_center_point.x = 0.0f;
      ave_center_point.y = 0.0f;
    }
  }

  return ave_center_point;
}

}  // namespace multi_sensor_mapping