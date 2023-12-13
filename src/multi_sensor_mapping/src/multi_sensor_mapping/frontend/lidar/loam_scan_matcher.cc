#include "multi_sensor_mapping/frontend/lidar/loam_scan_matcher.h"

#include <omp.h>
#include <pcl/common/angles.h>
#include <tf/tf.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "multi_sensor_mapping/factor/analytical_diff/loam_factor.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

LOAMScanMatcher::LOAMScanMatcher(double _corner_ds_leaf_size,
                                 double _surface_ds_leaf_size,
                                 int _corner_feature_min_valid_num,
                                 int _surface_feature_min_valid_num,
                                 int _num_cores, MatchMethod _match_method)
    : match_method_(_match_method),
      corner_ds_leaf_size_(_corner_ds_leaf_size),
      surface_ds_leaf_size_(_surface_ds_leaf_size),
      corner_feature_min_valid_num_(_corner_feature_min_valid_num),
      surface_feature_min_valid_num_(_surface_feature_min_valid_num),
      num_cores_(_num_cores),
      is_degenerate_(false),
      kdtree_corner_(new pcl::KdTreeFLANN<PointType>()),
      kdtree_surface_(new pcl::KdTreeFLANN<PointType>()),
      target_corner_cloud_(new CloudType),
      target_surface_cloud_(new CloudType),
      source_corner_cloud_(new CloudType),
      source_surface_cloud_(new CloudType),
      source_corner_cloud_ds_(new CloudType),
      source_surface_cloud_ds_(new CloudType),
      feature_cloud_original_(new CloudType),
      corff_sel_(new CloudType),
      opt_pose_q_(Eigen::Map<Eigen::Quaterniond>(pose_parameters_)),
      opt_pose_t_(Eigen::Map<Eigen::Vector3d>(pose_parameters_ + 4)) {
  corner_ds_filter_.setLeafSize(corner_ds_leaf_size_, corner_ds_leaf_size_,
                                corner_ds_leaf_size_);
  surface_ds_filter_.setLeafSize(surface_ds_leaf_size_, surface_ds_leaf_size_,
                                 surface_ds_leaf_size_);

  for (int i = 0; i < 6; ++i) {
    transform_to_be_mapped_[i] = 0;
  }
}

void LOAMScanMatcher::SetSourceFeature(CloudTypePtr &_corner_cloud,
                                       CloudTypePtr &_surface_cloud) {
  source_corner_cloud_ = _corner_cloud;
  source_surface_cloud_ = _surface_cloud;

  source_corner_cloud_ds_->clear();
  source_surface_cloud_ds_->clear();
  DownsampleCloud(source_corner_cloud_, source_corner_cloud_ds_);
  DownsampleCloud(source_surface_cloud_, source_surface_cloud_ds_, false);

  // std::cout << "source corner size : " << source_corner_cloud_->size() << " :
  // "
  //           << source_corner_cloud_ds_->size() << std::endl;
  // std::cout << "source surface size : " << source_surface_cloud_->size()
  //           << " : " << source_surface_cloud_ds_->size() << std::endl;
}

void LOAMScanMatcher::SetSourceFeature(VelRTPointCloudPtr &_corner_cloud,
                                       VelRTPointCloudPtr &_surface_cloud) {
  CloudConvert(_corner_cloud, source_corner_cloud_);
  CloudConvert(_surface_cloud, source_surface_cloud_);

  source_corner_cloud_ds_->clear();
  source_surface_cloud_ds_->clear();
  DownsampleCloud(source_corner_cloud_, source_corner_cloud_ds_);
  DownsampleCloud(source_surface_cloud_, source_surface_cloud_ds_, false);
}

void LOAMScanMatcher::SetTargetFeature(CloudTypePtr &_target_corner_cloud,
                                       CloudTypePtr &_target_surface_cloud) {
  target_corner_cloud_->clear();
  DownsampleCloud(_target_corner_cloud, target_corner_cloud_);
  target_surface_cloud_->clear();
  DownsampleCloud(_target_surface_cloud, target_surface_cloud_, false);

  kdtree_corner_->setInputCloud(target_corner_cloud_);
  kdtree_surface_->setInputCloud(target_surface_cloud_);
}

bool LOAMScanMatcher::Align(const Eigen::Matrix4d &_prediction,
                            Eigen::Matrix4d &_result) {
  if (target_corner_cloud_->empty() || target_surface_cloud_->empty())
    return false;

  if (source_corner_cloud_ds_->size() > corner_feature_min_valid_num_ &&
      source_surface_cloud_ds_->size() > surface_feature_min_valid_num_) {
    if (match_method_ == MatchMethod::MATCH_METHOD_LOAM) {
      UpdateInitialGuess(_prediction);
      Scan2MapOptimization();
      _result = Trans2Affine3f(transform_to_be_mapped_)
                    .matrix()
                    .template cast<double>();
      if (is_degenerate_) {
        AWARN_F("[LOAMScanMatcher] Scan Match Degenerate!");
      }
      return true;
    } else if (match_method_ == MatchMethod::MATCH_METHOD_FAST_LOAM) {
      UpdateInitialGuess(_prediction);
      FastScan2MapOptimization();
      _result = Eigen::Matrix4d::Identity();
      _result.block<3, 3>(0, 0) = opt_pose_q_.toRotationMatrix();
      _result.block<3, 1>(0, 3) = opt_pose_t_;
      return true;
    }
  }
  AINFO_F(
      "[LOAMScanMatcher] Not enough features! Corner feature size %zu, Surface "
      "feature size : %zu",
      source_corner_cloud_ds_->size(), source_surface_cloud_ds_->size());

  return false;
}

CloudTypePtr LOAMScanMatcher::GetTargetFeatureCloud() {
  CloudTypePtr feature_cloud(new CloudType);
  *feature_cloud += *target_corner_cloud_;
  *feature_cloud += *target_surface_cloud_;
  return feature_cloud;
}

void LOAMScanMatcher::DownsampleCloud(CloudTypePtr cloud_in,
                                      CloudTypePtr cloud_out, bool is_corner) {
  if (is_corner) {
    corner_ds_filter_.setInputCloud(cloud_in);
    corner_ds_filter_.filter(*cloud_out);
  } else {
    surface_ds_filter_.setInputCloud(cloud_in);
    surface_ds_filter_.filter(*cloud_out);
  }
}

void LOAMScanMatcher::CloudConvert(VelRTPointCloudPtr &input,
                                   CloudTypePtr &output) {
  output->clear();
  for (size_t i = 0; i < input->points.size(); i++) {
    PointType p;
    p.x = input->points[i].x;
    p.y = input->points[i].y;
    p.z = input->points[i].z;
    p.intensity = input->points[i].intensity;
    output->push_back(p);
  }
}

void LOAMScanMatcher::UpdateInitialGuess(const Eigen::Matrix4d &_prediction) {
  if (match_method_ == MatchMethod::MATCH_METHOD_LOAM) {
    double roll, pitch, yaw;
    tf::Matrix3x3(_prediction(0, 0), _prediction(0, 1), _prediction(0, 2),
                  _prediction(1, 0), _prediction(1, 1), _prediction(1, 2),
                  _prediction(2, 0), _prediction(2, 1), _prediction(2, 2))
        .getRPY(roll, pitch, yaw);

    transform_to_be_mapped_[0] = roll;
    transform_to_be_mapped_[1] = pitch;
    transform_to_be_mapped_[2] = yaw;
    transform_to_be_mapped_[3] = _prediction(0, 3);
    transform_to_be_mapped_[4] = _prediction(1, 3);
    transform_to_be_mapped_[5] = _prediction(2, 3);

    trans_point_associate_to_map_ = Trans2Affine3f(transform_to_be_mapped_);
  } else if (match_method_ == MatchMethod::MATCH_METHOD_FAST_LOAM) {
    Eigen::Quaterniond pose_q(_prediction.block<3, 3>(0, 0));
    pose_parameters_[0] = pose_q.x();
    pose_parameters_[1] = pose_q.y();
    pose_parameters_[2] = pose_q.z();
    pose_parameters_[3] = pose_q.w();
    pose_parameters_[4] = _prediction(0, 3);
    pose_parameters_[5] = _prediction(1, 3);
    pose_parameters_[6] = _prediction(2, 3);
    opt_pose_q_ = Eigen::Map<Eigen::Quaterniond>(pose_parameters_);
    opt_pose_t_ = Eigen::Map<Eigen::Vector3d>(pose_parameters_ + 4);

    latest_opt_pose_q_ = opt_pose_q_;
    latest_opt_pose_t_ = opt_pose_t_;
  }
}

Eigen::Affine3f LOAMScanMatcher::Trans2Affine3f(float transformIn[]) {
  return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5],
                                transformIn[0], transformIn[1], transformIn[2]);
}

void LOAMScanMatcher::UpdatePointAssociateToMap() {
  trans_point_associate_to_map_ = Trans2Affine3f(transform_to_be_mapped_);
}

void LOAMScanMatcher::PointAssociateToMap(PointType pi, PointType &po) {
  if (match_method_ == MatchMethod::MATCH_METHOD_LOAM) {
    po.x = trans_point_associate_to_map_(0, 0) * pi.x +
           trans_point_associate_to_map_(0, 1) * pi.y +
           trans_point_associate_to_map_(0, 2) * pi.z +
           trans_point_associate_to_map_(0, 3);
    po.y = trans_point_associate_to_map_(1, 0) * pi.x +
           trans_point_associate_to_map_(1, 1) * pi.y +
           trans_point_associate_to_map_(1, 2) * pi.z +
           trans_point_associate_to_map_(1, 3);
    po.z = trans_point_associate_to_map_(2, 0) * pi.x +
           trans_point_associate_to_map_(2, 1) * pi.y +
           trans_point_associate_to_map_(2, 2) * pi.z +
           trans_point_associate_to_map_(2, 3);
    po.intensity = pi.intensity;
  } else if (match_method_ == MatchMethod::MATCH_METHOD_FAST_LOAM) {
    Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
    Eigen::Vector3d point_w = opt_pose_q_ * point_curr + opt_pose_t_;
    po.x = point_w.x();
    po.y = point_w.y();
    po.z = point_w.z();
    po.intensity = pi.intensity;
  }
}

void LOAMScanMatcher::Scan2MapOptimization() {
  for (int iter_count = 0; iter_count < 30; iter_count++) {
    auto start_step1 = std::chrono::steady_clock::now();
    CornerOptimization();
    SurfaceOptimazation();
    CombineOptimazationCoeffs();
    auto end_step1 = std::chrono::steady_clock::now();
    auto duration_step1 = std::chrono::duration_cast<std::chrono::microseconds>(
        end_step1 - start_step1);

    auto start_step2 = std::chrono::steady_clock::now();
    bool optimize_success_flag = LMOptimization(iter_count);
    auto end_step2 = std::chrono::steady_clock::now();
    auto duration_step2 = std::chrono::duration_cast<std::chrono::microseconds>(
        end_step2 - start_step2);

    if (optimize_success_flag) break;
  }
}

void LOAMScanMatcher::FastScan2MapOptimization() {
  int ceres_opt_max_iter_num = 8;
  for (int iter_count = 0; iter_count < 2; iter_count++) {
    auto start_step1 = std::chrono::steady_clock::now();
    // 关联
    CornerFeatureAssociation();
    SurfaceFeatureAssociation();
    auto end_step1 = std::chrono::steady_clock::now();
    auto duration_step1 = std::chrono::duration_cast<std::chrono::microseconds>(
        end_step1 - start_step1);

    auto start_step2 = std::chrono::steady_clock::now();

    // 优化
    JointOptimization(ceres_opt_max_iter_num);
    auto end_step2 = std::chrono::steady_clock::now();
    auto duration_step2 = std::chrono::duration_cast<std::chrono::microseconds>(
        end_step2 - start_step2);

    if (CheckConverge(iter_count)) {
      break;
    }
  }
}

void LOAMScanMatcher::CornerOptimization() {
  UpdatePointAssociateToMap();

  corner_feature_original_vec_.resize(source_corner_cloud_ds_->size());
  corner_feature_coeff_vec_.resize(source_corner_cloud_ds_->size());
  corner_feature_origianl_flag_vec_.resize(source_corner_cloud_ds_->size());
  std::fill(corner_feature_origianl_flag_vec_.begin(),
            corner_feature_origianl_flag_vec_.end(), false);

#pragma omp parallel for num_threads(num_cores_)
  for (int i = 0; i < source_corner_cloud_ds_->size(); i++) {
    PointType pointOri, pointSel, coeff;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = source_corner_cloud_ds_->points[i];
    PointAssociateToMap(pointOri, pointSel);
    kdtree_corner_->nearestKSearch(pointSel, 5, pointSearchInd,
                                   pointSearchSqDis);

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    if (pointSearchSqDis[4] < 1.0) {
      float cx = 0, cy = 0, cz = 0;
      for (int j = 0; j < 5; j++) {
        cx += target_corner_cloud_->points[pointSearchInd[j]].x;
        cy += target_corner_cloud_->points[pointSearchInd[j]].y;
        cz += target_corner_cloud_->points[pointSearchInd[j]].z;
      }
      cx /= 5;
      cy /= 5;
      cz /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int j = 0; j < 5; j++) {
        float ax = target_corner_cloud_->points[pointSearchInd[j]].x - cx;
        float ay = target_corner_cloud_->points[pointSearchInd[j]].y - cy;
        float az = target_corner_cloud_->points[pointSearchInd[j]].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      matA1.at<float>(0, 0) = a11;
      matA1.at<float>(0, 1) = a12;
      matA1.at<float>(0, 2) = a13;
      matA1.at<float>(1, 0) = a12;
      matA1.at<float>(1, 1) = a22;
      matA1.at<float>(1, 2) = a23;
      matA1.at<float>(2, 0) = a13;
      matA1.at<float>(2, 1) = a23;
      matA1.at<float>(2, 2) = a33;

      cv::eigen(matA1, matD1, matV1);

      if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;
        float x1 = cx + 0.1 * matV1.at<float>(0, 0);
        float y1 = cy + 0.1 * matV1.at<float>(0, 1);
        float z1 = cz + 0.1 * matV1.at<float>(0, 2);
        float x2 = cx - 0.1 * matV1.at<float>(0, 0);
        float y2 = cy - 0.1 * matV1.at<float>(0, 1);
        float z2 = cz - 0.1 * matV1.at<float>(0, 2);

        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                              ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                          ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                              ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                          ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                              ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                         (z1 - z2) * (z1 - z2));

        float la =
            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
            a012 / l12;

        float lb =
            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float lc =
            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float ld2 = a012 / l12;

        float s = 1 - 0.9 * fabs(ld2);

        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        if (s > 0.1) {
          corner_feature_original_vec_[i] = pointOri;
          corner_feature_coeff_vec_[i] = coeff;
          corner_feature_origianl_flag_vec_[i] = true;
        }
      }
    }
  }
}

void LOAMScanMatcher::SurfaceOptimazation() {
  UpdatePointAssociateToMap();
  surface_feature_original_vec_.resize(source_surface_cloud_ds_->size());
  surface_feature_coeff_vec_.resize(source_surface_cloud_ds_->size());
  surface_feature_origianl_flag_vec_.resize(source_surface_cloud_ds_->size());
  std::fill(surface_feature_origianl_flag_vec_.begin(),
            surface_feature_origianl_flag_vec_.end(), false);

#pragma omp parallel for num_threads(num_cores_)
  for (int i = 0; i < source_surface_cloud_ds_->size(); i++) {
    PointType pointOri, pointSel, coeff;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = source_surface_cloud_ds_->points[i];
    PointAssociateToMap(pointOri, pointSel);
    kdtree_surface_->nearestKSearch(pointSel, 5, pointSearchInd,
                                    pointSearchSqDis);

    Eigen::Matrix<float, 5, 3> matA0;
    Eigen::Matrix<float, 5, 1> matB0;
    Eigen::Vector3f matX0;

    matA0.setZero();
    matB0.fill(-1);
    matX0.setZero();

    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = target_surface_cloud_->points[pointSearchInd[j]].x;
        matA0(j, 1) = target_surface_cloud_->points[pointSearchInd[j]].y;
        matA0(j, 2) = target_surface_cloud_->points[pointSearchInd[j]].z;
      }

      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * target_surface_cloud_->points[pointSearchInd[j]].x +
                 pb * target_surface_cloud_->points[pointSearchInd[j]].y +
                 pc * target_surface_cloud_->points[pointSearchInd[j]].z + pd) >
            0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) /
                          sqrt(sqrt(pointSel.x * pointSel.x +
                                    pointSel.y * pointSel.y +
                                    pointSel.z * pointSel.z));

        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        if (s > 0.1) {
          surface_feature_original_vec_[i] = pointOri;
          surface_feature_coeff_vec_[i] = coeff;
          surface_feature_origianl_flag_vec_[i] = true;
        }
      }
    }
  }
}

void LOAMScanMatcher::CombineOptimazationCoeffs() {
  feature_cloud_original_->clear();
  corff_sel_->clear();
  for (int i = 0; i < corner_feature_original_vec_.size(); ++i) {
    if (corner_feature_origianl_flag_vec_[i] == true) {
      feature_cloud_original_->push_back(corner_feature_original_vec_[i]);
      corff_sel_->push_back(corner_feature_coeff_vec_[i]);
    }
  }

  for (int i = 0; i < surface_feature_original_vec_.size(); ++i) {
    if (surface_feature_origianl_flag_vec_[i] == true) {
      feature_cloud_original_->push_back(surface_feature_original_vec_[i]);
      corff_sel_->push_back(surface_feature_coeff_vec_[i]);
    }
  }
}

bool LOAMScanMatcher::LMOptimization(int iter_count) {
  float srx = sin(transform_to_be_mapped_[1]);
  float crx = cos(transform_to_be_mapped_[1]);
  float sry = sin(transform_to_be_mapped_[2]);
  float cry = cos(transform_to_be_mapped_[2]);
  float srz = sin(transform_to_be_mapped_[0]);
  float crz = cos(transform_to_be_mapped_[0]);

  int laserCloudSelNum = feature_cloud_original_->size();
  if (laserCloudSelNum < 50) {
    return false;
  }

  cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

  PointType pointOri, coeff;

  for (int i = 0; i < laserCloudSelNum; i++) {
    pointOri.x = feature_cloud_original_->points[i].y;
    pointOri.y = feature_cloud_original_->points[i].z;
    pointOri.z = feature_cloud_original_->points[i].x;

    coeff.x = corff_sel_->points[i].y;
    coeff.y = corff_sel_->points[i].z;
    coeff.z = corff_sel_->points[i].x;
    coeff.intensity = corff_sel_->points[i].intensity;

    float arx =
        (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
         srx * sry * pointOri.z) *
            coeff.x +
        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) *
            coeff.y +
        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
         cry * srx * pointOri.z) *
            coeff.z;

    float ary =
        ((cry * srx * srz - crz * sry) * pointOri.x +
         (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) *
            coeff.x +
        ((-cry * crz - srx * sry * srz) * pointOri.x +
         (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) *
            coeff.z;

    float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                 (-cry * crz - srx * sry * srz) * pointOri.y) *
                    coeff.x +
                (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                ((sry * srz + cry * crz * srx) * pointOri.x +
                 (crz * sry - cry * srx * srz) * pointOri.y) *
                    coeff.z;

    matA.at<float>(i, 0) = arz;
    matA.at<float>(i, 1) = arx;
    matA.at<float>(i, 2) = ary;
    matA.at<float>(i, 3) = coeff.z;
    matA.at<float>(i, 4) = coeff.x;
    matA.at<float>(i, 5) = coeff.y;
    matB.at<float>(i, 0) = -coeff.intensity;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  cv::Mat matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

  if (iter_count == 0) {
    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    is_degenerate_ = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2.at<float>(i, j) = 0;
        }
        is_degenerate_ = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (is_degenerate_) {
    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transform_to_be_mapped_[0] += matX.at<float>(0, 0);
  transform_to_be_mapped_[1] += matX.at<float>(1, 0);
  transform_to_be_mapped_[2] += matX.at<float>(2, 0);
  transform_to_be_mapped_[3] += matX.at<float>(3, 0);
  transform_to_be_mapped_[4] += matX.at<float>(4, 0);
  transform_to_be_mapped_[5] += matX.at<float>(5, 0);

  float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                      pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                      pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
  float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                      pow(matX.at<float>(4, 0) * 100, 2) +
                      pow(matX.at<float>(5, 0) * 100, 2));

  if (deltaR < 0.05 && deltaT < 0.05) {
    return true;  // converged
  }
  return false;  // keep optimizing
}

void LOAMScanMatcher::CornerFeatureAssociation() {
  corner_feature_association_vec_.resize(source_corner_cloud_ds_->size());
  corner_feature_origianl_flag_vec_.resize(source_corner_cloud_ds_->size());
  std::fill(corner_feature_origianl_flag_vec_.begin(),
            corner_feature_origianl_flag_vec_.end(), false);

#pragma omp parallel for num_threads(num_cores_)
  for (int i = 0; i < source_corner_cloud_ds_->size(); i++) {
    PointType pointOri, pointSel;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = source_corner_cloud_ds_->points[i];
    PointAssociateToMap(pointOri, pointSel);
    kdtree_corner_->nearestKSearch(pointSel, 5, pointSearchInd,
                                   pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(target_corner_cloud_->points[pointSearchInd[j]].x,
                            target_corner_cloud_->points[pointSearchInd[j]].y,
                            target_corner_cloud_->points[pointSearchInd[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        corner_feature_association_vec_[i].ori_point = curr_point;
        corner_feature_association_vec_[i].point_a = point_a;
        corner_feature_association_vec_[i].point_b = point_b;
        corner_feature_origianl_flag_vec_[i] = true;
      }
    }
  }
}

void LOAMScanMatcher::SurfaceFeatureAssociation() {
  surface_feature_association_vec_.resize(source_surface_cloud_ds_->size());
  surface_feature_origianl_flag_vec_.resize(source_surface_cloud_ds_->size());
  std::fill(surface_feature_origianl_flag_vec_.begin(),
            surface_feature_origianl_flag_vec_.end(), false);

#pragma omp parallel for num_threads(num_cores_)
  for (int i = 0; i < source_surface_cloud_ds_->size(); i++) {
    PointType pointOri, pointSel;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = source_surface_cloud_ds_->points[i];
    PointAssociateToMap(pointOri, pointSel);
    kdtree_surface_->nearestKSearch(pointSel, 5, pointSearchInd,
                                    pointSearchSqDis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 =
        -1 * Eigen::Matrix<double, 5, 1>::Ones();

    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = target_surface_cloud_->points[pointSearchInd[j]].x;
        matA0(j, 1) = target_surface_cloud_->points[pointSearchInd[j]].y;
        matA0(j, 2) = target_surface_cloud_->points[pointSearchInd[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * target_surface_cloud_->points[pointSearchInd[j]].x +
                 norm(1) * target_surface_cloud_->points[pointSearchInd[j]].y +
                 norm(2) * target_surface_cloud_->points[pointSearchInd[j]].z +
                 negative_OA_dot_norm) > 0.2) {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
      if (planeValid) {
        surface_feature_association_vec_[i].ori_point = curr_point;
        surface_feature_association_vec_[i].plane_norm = norm;
        surface_feature_association_vec_[i].plane_dis = negative_OA_dot_norm;
        surface_feature_origianl_flag_vec_[i] = true;
      }
    }
  }
}

bool LOAMScanMatcher::JointOptimization(int max_iter_num) {
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  auto start_step1 = std::chrono::steady_clock::now();

  int corner_opt_count = 0;
  int surface_opt_count = 0;
  problem.AddParameterBlock(pose_parameters_, 7, new PoseSE3Parameterization());
  for (int i = 0; i < corner_feature_origianl_flag_vec_.size(); ++i) {
    if (corner_feature_origianl_flag_vec_[i] == true) {
      ceres::CostFunction *cost_function =
          new EdgeAnalyticFunctor(corner_feature_association_vec_[i].ori_point,
                                  corner_feature_association_vec_[i].point_a,
                                  corner_feature_association_vec_[i].point_b);
      problem.AddResidualBlock(cost_function, loss_function, pose_parameters_);
      corner_opt_count++;
    }
  }

  for (int i = 0; i < surface_feature_origianl_flag_vec_.size(); ++i) {
    if (surface_feature_origianl_flag_vec_[i] == true) {
      ceres::CostFunction *cost_function = new SurfNormAnalyticFunctor(
          surface_feature_association_vec_[i].ori_point,
          surface_feature_association_vec_[i].plane_norm,
          surface_feature_association_vec_[i].plane_dis);
      problem.AddResidualBlock(cost_function, loss_function, pose_parameters_);
      surface_opt_count++;
    }
  }

  // auto end_step1 = std::chrono::steady_clock::now();
  // auto duration_step1 =
  // std::chrono::duration_cast<std::chrono::microseconds>(
  //     end_step1 - start_step1);

  if (corner_opt_count < corner_feature_min_valid_num_ &&
      surface_opt_count < surface_feature_min_valid_num_) {
    AWARN_F("[LOAMScanMatcher] Not enough association [ %i , %i ] ",
            corner_opt_count, surface_opt_count);

    return false;
  }

  auto start_step2 = std::chrono::steady_clock::now();
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = max_iter_num;
  options.minimizer_progress_to_stdout = false;
  options.check_gradients = false;
  options.gradient_check_relative_precision = 1e-4;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  // std::cout << summary.BriefReport() << std::endl;

  // auto end_step2 = std::chrono::steady_clock::now();
  // auto duration_step2 =
  // std::chrono::duration_cast<std::chrono::microseconds>(
  //     end_step2 - start_step2);

  return true;
}

bool LOAMScanMatcher::CheckConverge(int iter_count) {
  if (iter_count < 2) {
    latest_opt_pose_q_ = opt_pose_q_;
    latest_opt_pose_t_ = opt_pose_t_;
    return false;
  }
  Eigen::Vector3d delta_translation = opt_pose_t_ - latest_opt_pose_t_;
  float deltaT = sqrt(pow(delta_translation(0) * 100, 2) +
                      pow(delta_translation(1) * 100, 2) +
                      pow(delta_translation(2) * 100, 2));
  float deltaR = sqrt(
      pow(pcl::rad2deg((pose_parameters_[0] - latest_opt_pose_q_.x()) * 2), 2) +
      pow(pcl::rad2deg((pose_parameters_[1] - latest_opt_pose_q_.y()) * 2), 2) +
      pow(pcl::rad2deg((pose_parameters_[2] - latest_opt_pose_q_.z()) * 2), 2));

  latest_opt_pose_q_ = opt_pose_q_;
  latest_opt_pose_t_ = opt_pose_t_;

  if (deltaT < 0.05 && deltaR < 0.05) {
    return true;
  }
  return false;
}

}  // namespace multi_sensor_mapping
