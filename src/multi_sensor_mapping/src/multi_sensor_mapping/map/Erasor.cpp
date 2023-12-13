#include "multi_sensor_mapping/map/Erasor.h"

namespace multi_sensor_mapping {

ERASOR::ERASOR()
    : max_range_(80),
      max_height_(3),
      min_height_(-1),
      num_ring_(20),
      num_sector_(60),
      num_lowest_pts_(5),
      minimum_num_pts_(5),
      scan_ratio_threshold_(0.2),
      bin_max_height_threshold_(0),
      num_initial_ground_seed_(10),
      distance_threshold_(0.125) {
  sector_resolution_ = 2 * PI / (double)num_sector_;
  ring_resolution_ = max_range_ / (double)num_ring_;

  InitRPOD(r_pod_map_);
  InitRPOD(r_pod_scan_);
  InitRPOD(r_pod_selected_);
}

void ERASOR::SetRawMapCloud(CloudTypePtr _map_cloud) {
  map_arranged_.reset(new CloudType);
  map_cloud_ = _map_cloud;
  // 对点云降采样
  utils::DownsampleCloudAdapted(*_map_cloud, *map_arranged_, 0.1);
}

void ERASOR::ScanFilter(const CloudTypePtr _source_cloud, const Transf &_pose) {
  if (!map_cloud_) {
    LOG(INFO) << "Please input raw map cloud first";
    return;
  }

  CloudTypePtr map_VOI(new CloudType);
  // 外部点云，本次不进行过滤
  CloudTypePtr map_outskirt(new CloudType);

  FetchVIO(_pose, map_VOI, map_outskirt);

  SetInputVOI(map_VOI, _source_cloud);

  CompareVOIandRevertGround();

  CloudType rpod_cloud;
  RPOD2Cloud(r_pod_selected_, rpod_cloud);

  CloudType static_cloud, static_cloud_in_map;
  static_cloud += rpod_cloud;
  static_cloud += ground_cloud_;
  static_cloud += map_complement_;

  pcl::transformPointCloud(static_cloud, static_cloud_in_map, _pose);

  map_arranged_->clear();
  static_cloud_in_map += *map_outskirt;

  utils::DownsampleCloudAdapted(static_cloud_in_map, *map_arranged_, 0.1);

  //  std::cout << rpod_cloud.size() << " : " << ground_cloud_.size() << " : "
  //            << map_complement_.size() << std::endl;
}

void ERASOR::FetchVIO(const Transf &_pose, CloudTypePtr _map_voi,
                      CloudTypePtr _map_outskirt) {
  if (!map_cloud_) {
    LOG(INFO) << "Please input raw map cloud first";
    return;
  }

  CloudTypePtr inlier_cloud(new CloudType);
  double max_range_square = std::pow(max_range_, 2);
  for (auto const &pt : map_arranged_->points) {
    double dist_square =
        pow(pt.x - _pose(0, 3), 2) + pow(pt.y - _pose(1, 3), 2);
    if (dist_square < max_range_square) {
      inlier_cloud->push_back(pt);
    } else {
      _map_outskirt->push_back(pt);
    }
  }

  _map_voi->clear();

  pcl::transformPointCloud(*inlier_cloud, *_map_voi, _pose.inverse());
}

void ERASOR::SetInputVOI(const CloudTypePtr _map_voi,
                         const CloudTypePtr _scan_voi) {
  for (int theta = 0; theta < num_sector_; ++theta) {
    for (int r = 0; r < num_ring_; ++r) {
      ClearBin(r_pod_map_[r][theta]);
      ClearBin(r_pod_scan_[r][theta]);
      ClearBin(r_pod_selected_[r][theta]);
    }
  }

  VOI2RPOD(_scan_voi, r_pod_scan_);
  VOI2RPOD(_map_voi, r_pod_map_, map_complement_);
}

void ERASOR::InitRPOD(R_POD &_r_pod) {
  if (!_r_pod.empty()) {
    _r_pod.clear();
  }

  Ring ring;
  Bin bin = {-INF, INF, 0, 0, false, static_cast<bool>(NOT_ASSIGNED)};
  bin.points.reserve(ENOUGH_NUM);
  for (int i = 0; i < num_sector_; i++) {
    ring.emplace_back(bin);
  }
  for (int j = 0; j < num_ring_; j++) {
    _r_pod.emplace_back(ring);
  }
}

void ERASOR::ClearBin(Bin &_bin) {
  _bin.max_h = -INF;
  _bin.min_h = INF;
  _bin.x = 0;
  _bin.y = 0;
  _bin.is_occupied = false;
  _bin.status = NOT_ASSIGNED;
  if (!_bin.points.empty()) _bin.points.clear();
}

void ERASOR::VOI2RPOD(const CloudTypePtr _cloud, R_POD &r_pod) {
  for (auto const &pt : _cloud->points) {
    if (pt.z < max_height_ && pt.z > min_height_) {
      double r = XY2Radius(pt.x, pt.y);
      if (r <= max_range_) {
        double theta = XY2Theta(pt.x, pt.y);

        int sector_idx = std::min(
            static_cast<int>((theta / sector_resolution_)), num_sector_ - 1);
        int ring_idx =
            std::min(static_cast<int>((r / ring_resolution_)), num_ring_ - 1);

        Point2Bin(pt, r_pod.at(ring_idx).at(sector_idx));
      }
    }
  }
}

void ERASOR::VOI2RPOD(const CloudTypePtr _cloud, R_POD &r_pod,
                      CloudType &_complement_cloud) {
  _complement_cloud.clear();
  for (auto const &pt : _cloud->points) {
    if (pt.z < max_height_ && pt.z > min_height_) {
      double r = XY2Radius(pt.x, pt.y);
      if (r <= max_range_) {
        double theta = XY2Theta(pt.x, pt.y);

        int sector_idx = std::min(
            static_cast<int>((theta / sector_resolution_)), num_sector_ - 1);
        int ring_idx =
            std::min(static_cast<int>((r / ring_resolution_)), num_ring_ - 1);

        Point2Bin(pt, r_pod.at(ring_idx).at(sector_idx));
      } else {
        _complement_cloud.push_back(pt);
      }
    } else {
      _complement_cloud.push_back(pt);
    }
  }
}

void ERASOR::Point2Bin(const PointType &_p, Bin &_bin) {
  _bin.is_occupied = true;
  _bin.points.push_back(_p);

  if (_p.z >= _bin.max_h) {
    _bin.max_h = _p.z;
    _bin.x = _p.x;
    _bin.y = _p.y;
  }
  if (_p.z <= _bin.min_h) {
    _bin.min_h = _p.z;
  }
}

void ERASOR::CompareVOIandRevertGround() {
  ground_cloud_.clear();

  for (int theta = 0; theta < num_sector_; theta++) {
    for (int r = 0; r < num_ring_; r++) {
      Bin &bin_scan = r_pod_scan_[r][theta];
      Bin &bin_map = r_pod_map_[r][theta];

      if (bin_scan.points.size() < minimum_num_pts_) {
        // 若Bin中点的数量小于阈值
        r_pod_selected_[r][theta] = bin_map;
        continue;
      }
      if (bin_scan.is_occupied && bin_map.is_occupied) {
        double map_h_diff = bin_map.max_h - bin_map.min_h;
        double scan_h_diff = bin_scan.max_h - bin_scan.min_h;
        double scan_ratio =
            std::min(map_h_diff / scan_h_diff, scan_h_diff / map_h_diff);

        /// 通过高度差寻找动态区域
        if (scan_ratio < scan_ratio_threshold_) {
          if (map_h_diff >= scan_h_diff) {
            if (bin_map.max_h > bin_max_height_threshold_) {
              r_pod_selected_[r][theta] = bin_scan;

              CloudType ground_cloud, non_ground_cloud;
              // 提取BIN中的地面点云
              ExtractGround(bin_map.points, ground_cloud, non_ground_cloud);
              r_pod_selected_[r][theta].points += ground_cloud;
              ground_cloud_ += ground_cloud;
            } else {
              r_pod_selected_[r][theta] = bin_map;
            }
          } else if (map_h_diff <= scan_h_diff) {
            r_pod_selected_[r][theta] = bin_map;
          }
        } else {
          /// 合并
          Bin merged_bin;
          MergeBins(bin_map, bin_scan, merged_bin);
          r_pod_selected_[r][theta] = merged_bin;
        }
      } else if (bin_scan.is_occupied) {
        r_pod_selected_[r][theta] = bin_scan;
      } else if (bin_map.is_occupied) {
        r_pod_selected_[r][theta] = bin_map;
      }
    }
  }
}

void ERASOR::ExtractGround(const CloudType &_source_cloud,
                           CloudType &_ground_cloud,
                           CloudType &_non_ground_cloud) {
  _ground_cloud.clear();
  _non_ground_cloud.clear();

  CloudType ground_pc, non_ground_pc;

  auto source_cloud_copy = _source_cloud;
  std::sort(source_cloud_copy.points.begin(), source_cloud_copy.points.end(),
            [](PointType a, PointType b) { return a.z < b.z; });

  /// Step1 过滤高度小于阈值的点云
  auto it = source_cloud_copy.points.begin();
  for (int i = 0; i < source_cloud_copy.points.size(); i++) {
    if (source_cloud_copy.points[i].z < min_height_) {
      it++;
    } else {
      break;
    }
  }
  source_cloud_copy.points.erase(source_cloud_copy.points.begin(), it);

  /// Step2 寻找高度最低的N个点为种子点
  ExtractInitialSeeds(source_cloud_copy, ground_pc);

  /// Step3 区域生长拟合平面
  Eigen::VectorXf normal;
  double th_dist_d, d;
  for (int i = 0; i < 3; i++) {
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(ground_pc, cov, pc_mean);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d = -(normal.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d = distance_threshold_ - d;

    ground_pc.clear();

    Eigen::MatrixXf points(_source_cloud.points.size(), 3);
    int j = 0;
    for (auto p : _source_cloud.points) {
      points.row(j++) << p.x, p.y, p.z;
    }

    Eigen::VectorXf result = points * normal;
    for (int r = 0; r < result.rows(); r++) {
      if (result[r] < th_dist_d) {
        ground_pc.push_back(_source_cloud.points[r]);
      } else {
        if (i == 2) {  // Last iteration
          non_ground_pc.push_back(_source_cloud.points[r]);
        }
      }
    }
  }

  _ground_cloud = ground_pc;
  _non_ground_cloud = non_ground_pc;
}

void ERASOR::ExtractInitialSeeds(const CloudType &_sorted_cloud,
                                 CloudType &_initial_seed) {
  _initial_seed.clear();

  double sum = 0;
  int cnt = 0;

  for (int i = num_lowest_pts_;
       i < _sorted_cloud.points.size() && cnt < num_initial_ground_seed_; i++) {
    sum += _sorted_cloud.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;

  for (int i = 0; i < _sorted_cloud.points.size(); i++) {
    if (_sorted_cloud.points[i].z < lpr_height + 0.5) {
      _initial_seed.push_back(_sorted_cloud.points[i]);
    }
  }
}

void ERASOR::MergeBins(const Bin &_src1, const Bin &_src2, Bin &_dst) {
  _dst.max_h = std::max(_src1.max_h, _src2.max_h);
  _dst.min_h = std::min(_src1.min_h, _src2.min_h);
  _dst.is_occupied = true;
  _dst.points.clear();
  for (auto const &pt : _src1.points) {
    _dst.points.push_back(pt);
  }
  for (auto const &pt : _src2.points) {
    _dst.points.push_back(pt);
  }
}

void ERASOR::RPOD2Cloud(const R_POD &_r_pod, CloudType &_cloud) {
  _cloud.clear();
  for (int theta = 0; theta < num_sector_; theta++) {
    for (int r = 0; r < num_ring_; r++) {
      if (_r_pod.at(r).at(theta).is_occupied) {
        for (auto const &pt : _r_pod.at(r).at(theta).points) {
          _cloud.points.push_back(pt);
        }
      }
    }
  }
}

double ERASOR::XY2Radius(const double &_x, const double &_y) {
  return sqrt(pow(_x, 2) + pow(_y, 2));
}

double ERASOR::XY2Theta(const double &_x, const double &_y) {
  if (_y >= 0) {
    return atan2(_y, _x);  // 1, 2 quadrant
  } else {
    return 2 * PI + atan2(_y, _x);  // 3, 4 quadrant
  }
}

}  // namespace multi_sensor_mapping
