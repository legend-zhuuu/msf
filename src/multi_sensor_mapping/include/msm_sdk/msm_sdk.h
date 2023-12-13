#ifndef MSM_SDK_H
#define MSM_SDK_H

#include "api_backend_process.h"
#include "api_imu_initializer.h"
#include "api_lidar_imu_mapping.h"
#include "api_lidar_imu_odometry.h"
#include "api_low_cost_location.h"
#include "api_low_cost_mapping.h"
#include "msm_types.h"

namespace msm_sdk {

/**
 * @brief 获取SDK版本号
 *
 * @return Version
 */
Version GetVersion();

/**
 * @brief Set the Log Path object 设置日志路径
 *
 * @param _log_path
 */
void SetLogPath(std::string _log_path);

}  // namespace msm_sdk

#endif