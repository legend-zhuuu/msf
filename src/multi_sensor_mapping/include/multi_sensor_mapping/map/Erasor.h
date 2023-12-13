/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef MSM_ERASOR_H
#define MSM_ERASOR_H

#include "multi_sensor_mapping/utils/UtilsPointcloud.h"

#define INF 10000000000000.0
#define PI 3.1415926535
#define ENOUGH_NUM 8000
#define NOT_ASSIGNED 0.0

namespace multi_sensor_mapping {

/**
 * @brief The Bin struct 栅格
 */
struct Bin {
  double max_h;
  double min_h;
  double x;
  double y;
  double status;
  bool is_occupied;

  CloudType points;
};

struct DynamicBinIdx {
  int r;
  int theta;
};

/**
 * @brief R_POD 占据状态描述子(Region-wise Pseudo Occupancy Descriptor)
 */
typedef std::vector<std::vector<Bin> > R_POD;
typedef std::vector<Bin> Ring;

/**
 * @brief The ERASOR class 基于栅格中点云分布差异的动态物体过滤方法
 * Adapted from ERASOR (https://github.com/LimHyungTae/ERASOR)
 */
class ERASOR {
 public:
  typedef std::shared_ptr<ERASOR> Ptr;

  ERASOR();

  /**
   * @brief SetRawMapCloud 输入原始地图点云
   * @param _map_cloud
   */
  void SetRawMapCloud(CloudTypePtr _map_cloud);

  /**
   * @brief ScanFilter 帧过滤
   * @param _source_cloud
   * @param _pose
   */
  void ScanFilter(const CloudTypePtr _source_cloud, const Transf& _pose);

  /**
   * @brief GetStaticMap 获取静态地图
   * @return
   */
  inline CloudTypePtr GetStaticMap() { return map_arranged_; }

 private:
  /**
   * @brief FetchVIO 提取VIO点云
   * @param _pose
   * @param _map_voi
   * @param _map_outskirt
   */
  void FetchVIO(const Transf& _pose, CloudTypePtr _map_voi,
                CloudTypePtr _map_outskirt);

  /**
   * @brief SetInputVOI 输入感兴趣栅格点云
   * @param _map_voi
   * @param _scan_voi
   */
  void SetInputVOI(const CloudTypePtr _map_voi, const CloudTypePtr _scan_voi);

  /**
   * @brief InitRPOD 初始化描述子
   * @param r_pod
   */
  void InitRPOD(R_POD& _r_pod);

  /**
   * @brief ClearBin 清除BIN
   * @param _bin
   */
  void ClearBin(Bin& _bin);

  /**
   * @brief VOI2RPOD 点云转化为RPOD
   * @param _cloud
   * @param r_pod
   */
  void VOI2RPOD(const CloudTypePtr _cloud, R_POD& r_pod);

  /**
   * @brief VOI2RPOD 点云转化为RPOD
   * @param _cloud
   * @param r_pod
   * @param _complement_cloud
   */
  void VOI2RPOD(const CloudTypePtr _cloud, R_POD& r_pod,
                CloudType& _complement_cloud);

  /**
   * @brief Point2Bin 点转换为Bin
   * @param _p
   * @param _bin
   */
  void Point2Bin(const PointType& _p, Bin& _bin);

  /**
   * @brief CompareVOIandRevertGround 比较VOI并还原地面
   */
  void CompareVOIandRevertGround();

  /**
   * @brief ExtractGround 地图提取
   * @param _source_cloud
   * @param _ground_cloud
   * @param _non_ground_cloud
   */
  void ExtractGround(const CloudType& _source_cloud, CloudType& _ground_cloud,
                     CloudType& _non_ground_cloud);

  /**
   * @brief ExtractInitialSeeds 提取初始种子点云
   * @param _sorted_cloud
   * @param _initial_seed
   */
  void ExtractInitialSeeds(const CloudType& _sorted_cloud,
                           CloudType& _initial_seed);

  /**
   * @brief MergeBins 合并Bin
   * @param _src1
   * @param _src2
   * @param _dst
   */
  void MergeBins(const Bin& _src1, const Bin& _src2, Bin& _dst);

  /**
   * @brief RPOD2Cloud RPOD转换为点云
   * @param _r_pod
   * @param _cloud
   */
  void RPOD2Cloud(const R_POD& _r_pod, CloudType& _cloud);

  /**
   * @brief XY2Radius 计算长度
   * @param _x
   * @param _y
   * @return
   */
  double XY2Radius(const double& _x, const double& _y);

  /**
   * @brief XY2Theta 计算夹角
   * @param _x
   * @param _y
   * @return
   */
  double XY2Theta(const double& _x, const double& _y);

 private:
  /// @brief 初始点云地图
  CloudTypePtr map_cloud_;
  /// @brief 更新过后的地图
  CloudTypePtr map_arranged_;

  /// @brief 动态分类参数
  std::vector<int> dynamic_classes_ = {252, 253, 254, 255, 256, 257, 258, 259};
  /// @brief 处理的最远距离
  double max_range_;
  /// @brief 最高高度
  double max_height_;
  /// @brief 最小高度
  double min_height_;
  /// @brief ring的数量
  int num_ring_;
  /// @brief 扇形数量
  int num_sector_;
  /// @brief
  int num_lowest_pts_;
  /// @brief Bin中最少点的数量
  int minimum_num_pts_;
  /// @brief 扇形分辨率
  double sector_resolution_;
  /// @brief ring分辨率
  double ring_resolution_;
  /// @brief 阈值
  double scan_ratio_threshold_;
  /// @brief Bin中的高度阈值
  double bin_max_height_threshold_;
  /// @brief 初始化地图种子的数量
  int num_initial_ground_seed_;
  /// @brief 距离阈值
  double distance_threshold_;

  /// @brief 地图的R-POD
  R_POD r_pod_map_;
  /// @brief 当前帧的R-POD
  R_POD r_pod_scan_;
  /// @brief 选中的R-POD
  R_POD r_pod_selected_;

  /// @brief 补全点云
  CloudType map_complement_;
  /// @brief 地面点云
  CloudType ground_cloud_;
};

}  // namespace multi_sensor_mapping

#endif
