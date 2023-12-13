#include "multi_sensor_mapping/frontend/lidar/patchwork_pp.h"

#include <pcl/segmentation/sac_segmentation.h>

namespace multi_sensor_mapping {

double XY2Radius(const double& x, const double& y) {
  return sqrt(x * x + y * y);
}

double XYTtheta(const double& x, const double& y) {  // 0 ~ 2 * PI

  double angle = atan2(y, x);
  return angle > 0 ? angle : 2 * M_PI + angle;
}

double PointDisSqr(const PointType& p) { return (p.x * p.x + p.y * p.y); }

double Point2PlaneDistance(Eigen::Vector3d& pt, Eigen::Vector4d& plane_coeff) {
  Eigen::Vector3d normal = plane_coeff.head<3>();
  double dist = pt.dot(normal) + plane_coeff(3);
  dist = dist > 0 ? dist : -dist;

  return dist;
}
void CalcMeanStdev(std::vector<double> vec, double& mean, double& stdev) {
  if (vec.size() <= 1) return;

  mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

  for (int i = 0; i < vec.size(); i++) {
    stdev += (vec.at(i) - mean) * (vec.at(i) - mean);
  }
  stdev /= vec.size() - 1;
  stdev = sqrt(stdev);
}

bool PointZCmp(PointType a, PointType b) { return a.z < b.z; }

PatchworkPP::PatchworkPP(
    const std::shared_ptr<PatchWorkParams>& _patchwork_param)
    : param_(_patchwork_param) {
  num_zone_ = 4;

  double min_range = param_->min_r;
  double min_range_z2 = (7 * param_->min_r + param_->max_r) / 8.0;
  double min_range_z3 = (3 * param_->min_r + param_->max_r) / 4.0;
  double min_range_z4 = (param_->min_r + param_->max_r) / 2.0;

  min_ranges_ = {min_range, min_range_z2, min_range_z3, min_range_z4};
  num_rings_each_zone_ = {2, 4, 4, 2};
  num_sectors_each_zone_ = {16, 32, 54, 32};
  elevation_thr_ = {0, 0, 0, 0};
  flatness_thr_ = {0, 0, 0, 0};

  num_rings_of_interest_ = elevation_thr_.size();

  ring_sizes_ = {(min_range_z2 - min_range) / num_rings_each_zone_.at(0),
                 (min_range_z3 - min_range_z2) / num_rings_each_zone_.at(1),
                 (min_range_z4 - min_range_z3) / num_rings_each_zone_.at(2),
                 (param_->max_r - min_range_z4) / num_rings_each_zone_.at(3)};

  sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0),
                   2 * M_PI / num_sectors_each_zone_.at(1),
                   2 * M_PI / num_sectors_each_zone_.at(2),
                   2 * M_PI / num_sectors_each_zone_.at(3)};

  for (size_t i = 0; i < num_zone_; i++) {
    Zone z;
    InitializeZone(z, num_sectors_each_zone_[i], num_rings_each_zone_[i]);
    concentric_zone_model_.push_back(z);
  }
}

void PatchworkPP::EstimateGround(CloudType _cloud_in, CloudType& _cloud_ground,
                                 CloudType& _cloud_nonground) {
  revert_pc_.clear();
  reject_pc_.clear();
  noise_pc_.clear();
  vertical_pc_.clear();

  _cloud_ground.clear();
  _cloud_nonground.clear();

  CloudType regionwise_ground;
  CloudType regionwise_nonground;

  /// Step1 Reflected Noise Removal (RNR)
  if (param_->enable_RNR) {
    ReflectedNoiseRemoval(_cloud_in, _cloud_nonground);
  }

  /// Step2 Concentric Zone Model (CZM)
  FlushPatches(concentric_zone_model_);
  Cloud2CZM(_cloud_in, concentric_zone_model_);

  int concentric_idx = 0;

  std::vector<RevertCandidate> candidates;
  std::vector<double> ringwise_flatness;

  for (int zone_idx = 0; zone_idx < num_zone_; ++zone_idx) {
    auto zone = concentric_zone_model_[zone_idx];

    for (int ring_idx = 0; ring_idx < num_rings_each_zone_[zone_idx];
         ++ring_idx) {
      for (int sector_idx = 0; sector_idx < num_sectors_each_zone_[zone_idx];
           ++sector_idx) {
        // 点数量判断
        if (zone[ring_idx][sector_idx].points.size() < param_->num_min_pts) {
          _cloud_nonground += zone[ring_idx][sector_idx];
          continue;
        }

        // 根据Z轴排序
        std::sort(zone[ring_idx][sector_idx].points.begin(),
                  zone[ring_idx][sector_idx].points.end(), PointZCmp);

        ExtractPiecewiseGround(zone_idx, zone[ring_idx][sector_idx],
                               regionwise_ground, regionwise_nonground);

        const double ground_uprightness = normal_(2);
        const double ground_elevation = pc_mean_(2, 0);
        const double ground_flatness = singular_values_.minCoeff();
        const double line_variable =
            singular_values_(1) != 0 ? singular_values_(0) / singular_values_(1)
                                     : std::numeric_limits<double>::max();

        double heading = 0.0;
        for (int i = 0; i < 3; i++) {
          heading += pc_mean_(i, 0) * normal_(i);
        }

        bool is_upright = ground_uprightness > param_->uprightness_thr;
        //        bool is_not_elevated =
        //            ground_elevation < elevation_thr_[concentric_idx];
        bool is_not_elevated = ground_elevation < -0.9 * param_->sensor_height;
        bool is_flat = ground_flatness < flatness_thr_[concentric_idx];
        bool is_near_zone = concentric_idx < num_rings_of_interest_;
        bool is_heading_outside = heading < 0.0;

        if (is_upright && is_not_elevated && is_near_zone) {
          update_elevation_[concentric_idx].push_back(ground_elevation);
          update_flatness_[concentric_idx].push_back(ground_flatness);

          ringwise_flatness.push_back(ground_flatness);
        }

        if (!is_upright) {
          _cloud_nonground += regionwise_ground;
        } else if (!is_near_zone) {
          //          _cloud_ground += regionwise_ground;
        } else if (!is_heading_outside) {
          _cloud_nonground += regionwise_ground;
        } else if (is_not_elevated) {
          _cloud_ground += regionwise_ground;
        } else {
          RevertCandidate candidate(concentric_idx, sector_idx, ground_flatness,
                                    line_variable, pc_mean_, regionwise_ground);
          candidates.push_back(candidate);
        }

        _cloud_nonground += regionwise_nonground;
      }

      if (!candidates.empty()) {
        if (param_->enable_TGR) {
          TemporalGroundRevert(_cloud_ground, _cloud_nonground,
                               ringwise_flatness, candidates, concentric_idx);
        } else {
          for (size_t i = 0; i < candidates.size(); i++) {
            _cloud_nonground += candidates[i].regionwise_ground;
          }
        }

        candidates.clear();
        ringwise_flatness.clear();
      }

      concentric_idx++;
    }
  }

  UpdateElevationThr();
  UpdateFlatnessThr();
}

void PatchworkPP::EstimateGround(CloudType _cloud_in,
                                 CloudType _init_ground_cloud,
                                 CloudType& _cloud_ground,
                                 CloudType& _cloud_nonground,
                                 double _sample_radius) {
  Eigen::Vector4d plane_coeffs;
  if (!FittingPlane(_init_ground_cloud, plane_coeffs)) {
    std::cout << "Fiting plane Failed " << std::endl;
    return;
  }

  _cloud_ground.clear();
  _cloud_nonground.clear();

  double radius_sqr = _sample_radius * _sample_radius;

  for (size_t i = 0; i < _cloud_in.points.size(); i++) {
    if (PointDisSqr(_cloud_in.points[i]) > radius_sqr) {
      continue;
    }

    Eigen::Vector3d point_vec(_cloud_in.points[i].x, _cloud_in.points[i].y,
                              _cloud_in.points[i].z);
    if (Point2PlaneDistance(point_vec, plane_coeffs) < 0.1) {
      _cloud_ground.push_back(_cloud_in.points[i]);
    } else {
      _cloud_nonground.push_back(_cloud_in.points[i]);
    }
  }
}

void PatchworkPP::ReflectedNoiseRemoval(CloudType& _cloud_in,
                                        CloudType& _cloud_nonground) {
  for (size_t i = 0; i < _cloud_in.size(); i++) {
    double r =
        sqrt(_cloud_in[i].x * _cloud_in[i].x + _cloud_in[i].y * _cloud_in[i].y);
    double z = _cloud_in[i].z;
    double ver_angle_in_deg = atan2(z, r) * 180 / M_PI;

    if (ver_angle_in_deg < param_->RNR_ver_angle_thr &&
        z < -param_->sensor_height - 0.8 &&
        _cloud_in[i].intensity < param_->RNR_intensity_thr) {
      noise_pc_.points.emplace_back(_cloud_in[i]);
      _cloud_in.points[i].z = std::numeric_limits<double>::min();
    }
  }

  _cloud_nonground += noise_pc_;
}

void PatchworkPP::InitializeZone(Zone& _z, int _num_sectors, int _num_rings) {
  _z.clear();
  CloudType cloud;
  cloud.reserve(1000);
  Ring ring;
  for (int i = 0; i < _num_sectors; i++) {
    ring.emplace_back(cloud);
  }
  for (int j = 0; j < _num_rings; j++) {
    _z.emplace_back(ring);
  }
}

void PatchworkPP::FlushPatches(std::vector<Zone>& czm) {
  for (int k = 0; k < num_zone_; k++) {
    for (int i = 0; i < num_rings_each_zone_[k]; i++) {
      for (int j = 0; j < num_sectors_each_zone_[k]; j++) {
        if (!czm[k][i][j].points.empty()) {
          czm[k][i][j].points.clear();
        }
      }
    }
  }
}

void PatchworkPP::Cloud2CZM(const CloudType& _src, std::vector<Zone>& _czm) {
  for (auto const& pt : _src.points) {
    if (pt.z == std::numeric_limits<double>::min()) continue;
    double r = XY2Radius(pt.x, pt.y);

    if ((r <= param_->max_r) && (r > param_->min_r)) {
      double theta = XYTtheta(pt.x, pt.y);

      int zone_idx = 0;
      if (r < min_ranges_[1])
        zone_idx = 0;
      else if (r < min_ranges_[2])
        zone_idx = 1;
      else if (r < min_ranges_[3])
        zone_idx = 2;
      else
        zone_idx = 3;

      int ring_idx = std::min(static_cast<int>(((r - min_ranges_[zone_idx]) /
                                                ring_sizes_[zone_idx])),
                              num_rings_each_zone_[zone_idx] - 1);
      int sector_idx =
          std::min(static_cast<int>((theta / sector_sizes_[zone_idx])),
                   num_sectors_each_zone_[zone_idx] - 1);

      _czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
    }
  }
}

void PatchworkPP::ExtractPiecewiseGround(const int zone_idx,
                                         const CloudType& src, CloudType& dst,
                                         CloudType& non_ground_dst) {
  CloudType ground_pc;
  if (!dst.empty()) {
    dst.clear();
  }
  if (!non_ground_dst.empty()) {
    non_ground_dst.clear();
  }

  CloudType src_wo_verticals;
  src_wo_verticals = src;

  if (param_->enable_RVPF) {
    for (int i = 0; i < param_->num_iter; i++) {
      ExtractInitialSeeds(zone_idx, src_wo_verticals, ground_pc,
                          param_->th_seeds_v);
      EstimatePlane(ground_pc);

      if (zone_idx == 0 && normal_(2) < param_->uprightness_thr) {
        CloudType src_tmp;
        src_tmp = src_wo_verticals;
        src_wo_verticals.clear();

        Eigen::MatrixXf points(src_tmp.points.size(), 3);
        int j = 0;
        for (auto& p : src_tmp.points) {
          points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;

        for (int r = 0; r < result.rows(); r++) {
          if (result[r] < param_->th_dist_v - d_ &&
              result[r] > -param_->th_dist_v - d_) {
            non_ground_dst.points.push_back(src_tmp[r]);
            vertical_pc_.points.push_back(src_tmp[r]);
          } else {
            src_wo_verticals.points.push_back(src_tmp[r]);
          }
        }
      } else {
        break;
      }
    }
  }

  ExtractInitialSeeds(zone_idx, src_wo_verticals, ground_pc);
  EstimatePlane(ground_pc);

  Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
  int j = 0;
  for (auto& p : src_wo_verticals.points) {
    points.row(j++) << p.x, p.y, p.z;
  }

  for (int i = 0; i < param_->num_iter; i++) {
    ground_pc.clear();

    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (i < param_->num_iter - 1) {
        if (result[r] < param_->th_dist - d_) {
          ground_pc.points.push_back(src_wo_verticals[r]);
        }
      } else {  // Final stage
        if (result[r] < param_->th_dist - d_) {
          dst.points.push_back(src_wo_verticals[r]);
        } else {
          non_ground_dst.points.push_back(src_wo_verticals[r]);
        }
      }
    }

    if (i < param_->num_iter - 1)
      EstimatePlane(ground_pc);
    else
      EstimatePlane(dst);
  }
}

void PatchworkPP::ExtractInitialSeeds(const int zone_idx,
                                      const CloudType& p_sorted,
                                      CloudType& init_seeds, double th_seed) {
  init_seeds.points.clear();
  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < p_sorted.points.size(); i++) {
      if (p_sorted.points[i].z <
          param_->adaptive_seed_selection_margin * param_->sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (int i = init_idx; i < p_sorted.points.size() && cnt < param_->num_lpr;
       i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + th_seed) {
      init_seeds.points.push_back(p_sorted.points[i]);
    }
  }
}

void PatchworkPP::ExtractInitialSeeds(const int zone_idx,
                                      const CloudType& p_sorted,
                                      CloudType& init_seeds) {
  init_seeds.points.clear();

  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < p_sorted.points.size(); i++) {
      if (p_sorted.points[i].z <
          param_->adaptive_seed_selection_margin * param_->sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (int i = init_idx; i < p_sorted.points.size() && cnt < param_->num_lpr;
       i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + param_->th_seeds) {
      init_seeds.points.push_back(p_sorted.points[i]);
    }
  }
}

void PatchworkPP::EstimatePlane(const CloudType& ground) {
  pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      cov_, Eigen::DecompositionOptions::ComputeFullU);
  singular_values_ = svd.singularValues();

  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));

  if (normal_(2) < 0) {
    for (int i = 0; i < 3; i++) normal_(i) *= -1;
  }

  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

void PatchworkPP::TemporalGroundRevert(CloudType& cloud_ground,
                                       CloudType& cloud_nonground,
                                       std::vector<double> ring_flatness,
                                       std::vector<RevertCandidate> candidates,
                                       int concentric_idx) {
  double mean_flatness = 0.0, stdev_flatness = 0.0;
  CalcMeanStdev(ring_flatness, mean_flatness, stdev_flatness);

  for (size_t i = 0; i < candidates.size(); i++) {
    RevertCandidate candidate = candidates[i];

    double mu_flatness = mean_flatness + 1.5 * stdev_flatness;
    double prob_flatness =
        1 / (1 + exp((candidate.ground_flatness - mu_flatness) /
                     (mu_flatness / 10)));

    if (candidate.regionwise_ground.size() > 1500 &&
        candidate.ground_flatness < param_->th_dist * param_->th_dist)
      prob_flatness = 1.0;

    double prob_line = 1.0;
    if (candidate.line_variable >
        8.0)  //&& candidate.line_dir > M_PI/4)// candidate.ground_elevation >
              // elevation_thr_[concentric_idx])
    {
      // if (verbose_) cout << "line_dir: " << candidate.line_dir << endl;
      prob_line = 0.0;
    }

    bool revert = prob_line * prob_flatness > 0.5;

    if (concentric_idx < num_rings_of_interest_) {
      if (revert) {
        revert_pc_ += candidate.regionwise_ground;
        cloud_ground += candidate.regionwise_ground;
      } else {
        reject_pc_ += candidate.regionwise_ground;
        cloud_nonground += candidate.regionwise_ground;
      }
    }
  }
}

void PatchworkPP::UpdateElevationThr() {
  for (int i = 0; i < num_rings_of_interest_; i++) {
    if (update_elevation_[i].empty()) continue;

    double update_mean = 0.0, update_stdev = 0.0;
    CalcMeanStdev(update_elevation_[i], update_mean, update_stdev);
    if (i == 0) {
      elevation_thr_[i] = update_mean + 3 * update_stdev;
      param_->sensor_height = -update_mean;
    } else
      elevation_thr_[i] = update_mean + 2 * update_stdev;

    //    std::cout << "elevation threshold [" << i << "]: " <<
    //    elevation_thr_[i]
    //              << std::endl;

    int exceed_num = update_elevation_[i].size() - 1000;
    if (exceed_num > 0)
      update_elevation_[i].erase(update_elevation_[i].begin(),
                                 update_elevation_[i].begin() + exceed_num);
  }

  return;
}

void PatchworkPP::UpdateFlatnessThr() {
  for (int i = 0; i < num_rings_of_interest_; i++) {
    if (update_flatness_[i].empty()) break;
    if (update_flatness_[i].size() <= 1) break;

    double update_mean = 0.0, update_stdev = 0.0;
    CalcMeanStdev(update_flatness_[i], update_mean, update_stdev);
    flatness_thr_[i] = update_mean + update_stdev;

    //    std::cout << "flatness threshold [" << i << "]: " << flatness_thr_[i]
    //              << std::endl;

    int exceed_num = update_flatness_[i].size() - 1000;
    if (exceed_num > 0)
      update_flatness_[i].erase(update_flatness_[i].begin(),
                                update_flatness_[i].begin() + exceed_num);
  }
}

bool PatchworkPP::FittingPlane(CloudType _cloud, Eigen::Vector4d& _coeffs) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;  /// Create the segmentation object
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);

  seg.setInputCloud(_cloud.makeShared());
  seg.segment(*inliers, *coefficients);

  for (int i = 0; i < 4; i++) {
    _coeffs(i) = coefficients->values[i];
  }

  if (inliers->indices.size() < 20) {
    return false;
  }

  return true;
}

}  // namespace multi_sensor_mapping
