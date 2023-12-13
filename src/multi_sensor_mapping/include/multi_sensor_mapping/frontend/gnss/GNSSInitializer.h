/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef MSM_GNSS_INITIALIZER_H
#define MSM_GNSS_INITIALIZER_H

#include <ceres/ceres.h>

#include "multi_sensor_mapping/utils/UtilsSensorData.h"

namespace multi_sensor_mapping {

class GNSSInitializer {
 public:
  typedef std::shared_ptr<GNSSInitializer> Ptr;

  /**
   * @brief GNSSInitializer
   */
  GNSSInitializer();

  /**
   * @brief GNSSAlign
   * @param _gnss_pose_data
   * @return
   */
  bool GNSSAlign(
      const std::vector<std::pair<GNSSData, Transf>> _gnss_pose_data);

  /**
   * @brief GNSSAlign4D 4自由度对齐
   * @param _gnss_pose_data
   * @return
   */
  bool GNSSAlign4D(
      const std::vector<std::pair<GNSSData, Transf>> _gnss_pose_data);

  /**
   * @brief GNSSAlign5D 5自由度对齐
   * @param _gnss_pose_data
   * @return
   */
  bool GNSSAlign5D(
      const std::vector<std::pair<GNSSData, Transf>> _gnss_pose_data);

  /**
   * @brief CheckAlignResult 检测对齐结果
   * @param _gnss_pose_data
   * @return
   */
  bool CheckAlignResult(
      const std::vector<std::pair<GNSSData, Transf>> _gnss_pose_data);

  /**
   * @brief PrintResult  结果显示
   */
  void PrintResult();

  /**
   * @brief GetPosition
   * @return
   */
  inline const Eigen::Vector3d& GetPosition() {
    return map_in_local_ENU_position_;
  }

  /**
   * @brief GetRotation
   * @return
   */
  inline const Eigen::Quaterniond& GetRotation() {
    return map_in_local_ENU_orientation_;
  }

 private:
  /// @brief 外参初始化成功标志位
  bool ext_initialized_flag_;
  /// @brief 地图坐标系在local ENU坐标系的位置
  Eigen::Vector3d map_in_local_ENU_position_;
  /// @brief 地图坐标系在local ENU坐标系的旋转
  Eigen::Quaterniond map_in_local_ENU_orientation_;
};

}  // namespace multi_sensor_mapping

#endif
