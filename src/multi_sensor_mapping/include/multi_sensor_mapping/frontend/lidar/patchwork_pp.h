#ifndef MSM_PATCHWORK_PP_H
#define MSM_PATCHWORK_PP_H

#include <memory>

#include "multi_sensor_mapping/param/patchwork_params.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

struct RevertCandidate {
  int concentric_idx;
  int sector_idx;
  double ground_flatness;
  double line_variable;
  Eigen::Vector4f pc_mean;
  CloudType regionwise_ground;

  RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var,
                  Eigen::Vector4f _pc_mean, CloudType _ground)
      : concentric_idx(_c_idx),
        sector_idx(_s_idx),
        ground_flatness(_flatness),
        line_variable(_line_var),
        pc_mean(_pc_mean),
        regionwise_ground(_ground) {}
};

/**
 * @brief The PatchworkPP class 地面估计器
 * Adapted from patchwork-plusplus
 * (https://github.com/url-kaist/patchwork-plusplus)
 */
class PatchworkPP {
 public:
  typedef std::vector<CloudType> Ring;
  typedef std::vector<Ring> Zone;

  /**
   * @brief PatchworkPP
   */
  PatchworkPP(const std::shared_ptr<PatchWorkParams>& _patchwork_param);

  /**
   * @brief EstimateGround 地面估计
   * @param _cloud_in
   * @param _cloud_ground
   * @param _cloud_nonground
   */
  void EstimateGround(CloudType _cloud_in, CloudType& _cloud_ground,
                      CloudType& _cloud_nonground);

  /**
   * @brief EstimateGround 地面估计(利用RANSAC)
   * @param _cloud_in
   * @param init_ground_cloud
   * @param _cloud_ground
   * @param _cloud_nonground
   * @param _sample_radius
   */
  void EstimateGround(CloudType _cloud_in, CloudType init_ground_cloud,
                      CloudType& _cloud_ground, CloudType& _cloud_nonground,
                      double _sample_radius);

 private:
  /**
   * @brief ReflectedNoiseRemoval 反射噪声滤除
   * @param _cloud_in
   * @param _cloud_ground
   */
  void ReflectedNoiseRemoval(CloudType& _cloud_in, CloudType& _cloud_ground);

  /**
   * @brief InitializeZone 初始化空间
   * @param _z
   * @param _num_sectors
   * @param _num_rings
   */
  void InitializeZone(Zone& _z, int _num_sectors, int _num_rings);

  /**
   * @brief FlushPatches  清空zone
   * @param czm
   */
  void FlushPatches(std::vector<Zone>& czm);

  /**
   * @brief Cloud2CZM
   * @param _src
   * @param _czm
   */
  void Cloud2CZM(const CloudType& _src, std::vector<Zone>& _czm);

  /**
   * @brief ExtractPiecewiseGround 分段地面提取
   * @param zone_idx
   * @param src
   * @param dst
   * @param non_ground_dst
   */
  void ExtractPiecewiseGround(const int zone_idx, const CloudType& src,
                              CloudType& dst, CloudType& non_ground_dst);

  /**
   * @brief ExtractInitialSeeds
   * @param zone_idx
   * @param p_sorted
   * @param init_seeds
   * @param th_seed
   */
  void ExtractInitialSeeds(const int zone_idx, const CloudType& p_sorted,
                           CloudType& init_seeds, double th_seed);

  /**
   * @brief ExtractInitialSeeds
   * @param zone_idx
   * @param p_sorted
   * @param init_seeds
   */
  void ExtractInitialSeeds(const int zone_idx, const CloudType& p_sorted,
                           CloudType& init_seeds);

  /**
   * @brief EstimatePlane 平面估计
   * @param _ground
   */
  void EstimatePlane(const CloudType& _ground);

  /**
   * @brief TemporalGroundRevert
   * @param cloud_ground
   * @param cloud_nonground
   * @param ring_flatness
   * @param candidates
   * @param concentric_idx
   */
  void TemporalGroundRevert(CloudType& cloud_ground, CloudType& cloud_nonground,
                            std::vector<double> ring_flatness,
                            std::vector<RevertCandidate> candidates,
                            int concentric_idx);

  /**
   * @brief UpdateElevationThr
   */
  void UpdateElevationThr();

  /**
   * @brief UpdateFlatnessThr
   */
  void UpdateFlatnessThr();

  /**
   * @brief FittingPlane 平面拟合
   * @param _cloud
   * @param _coeffs
   */
  bool FittingPlane(CloudType _cloud, Eigen::Vector4d& _coeffs);

 private:
  /// @brief 参数
  std::shared_ptr<PatchWorkParams> param_;

  /// @brief 空间数量
  int num_zone_;
  std::vector<double> sector_sizes_;
  std::vector<double> ring_sizes_;
  std::vector<double> min_ranges_;

  std::vector<int> num_sectors_each_zone_;
  std::vector<int> num_rings_each_zone_;
  std::vector<double> elevation_thr_;
  std::vector<double> flatness_thr_;
  int num_rings_of_interest_;

  /// @brief 同心圆模型
  std::vector<Zone> concentric_zone_model_;

  float d_;
  Eigen::VectorXf normal_;
  Eigen::MatrixXf pnormal_;
  Eigen::VectorXf singular_values_;
  Eigen::Matrix3f cov_;
  Eigen::Vector4f pc_mean_;

  /// @brief 噪声点云(RNR)
  CloudType noise_pc_;
  CloudType vertical_pc_;
  CloudType revert_pc_;
  CloudType reject_pc_;

  std::vector<double> update_flatness_[4];
  std::vector<double> update_elevation_[4];
};

}  // namespace multi_sensor_mapping

#endif
