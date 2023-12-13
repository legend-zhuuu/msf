#ifndef MSM_GNSS_CONVERTER_H
#define MSM_GNSS_CONVERTER_H

#include <GeographicLib/LocalCartesian.hpp>

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

/**
 * @brief The GNSSConverter class GNSS数据转换器(单例)
 */
class GNSSConverter {
 public:
  static GNSSConverter *GetInstance();

  /**
   * @brief InitOriginPosition 初始化位置
   * @param _gnss_data
   */
  void InitOriginPosition(GNSSData &_gnss_data);

  /**
   * @brief InitOriginPosition 初始化位置
   * @param lat
   * @param lon
   * @param alt
   */
  void InitOriginPosition(const double &lat, const double &lon,
                          const double &alt);

  /**
   * @brief UpdateForward 前向更新（LLA -> Local ENU）
   * @param _gnss_data
   */
  void UpdateForward(GNSSData &_gnss_data);

  /**
   * @brief UpdateForward 前向更新（LLA -> Local ENU）
   * @param lat
   * @param lon
   * @param alt
   * @param local_E
   * @param local_N
   * @param local_U
   */
  void UpdateForward(const double &lat, const double &lon, const double &alt,
                     double &local_E, double &local_N, double &local_U);

  /**
   * @brief UpdateReverse 反向更新 (Local ENU -> LLA)
   * @param _gnss_data
   */
  void UpdateReverse(GNSSData &_gnss_data);

  /**
   * @brief UpdateReverse 反向更新 (Local ENU -> LLA)
   * @param local_E
   * @param local_N
   * @param local_U
   * @param lat
   * @param lon
   * @param alt
   */
  void UpdateReverse(const double &local_E, const double &local_N,
                     const double &local_U, double &lat, double &lon,
                     double &alt);

  /**
   * @brief isConverterinitialized 是否初始化成功
   * @return
   */
  inline bool isConverterinitialized() {
    return origin_position_initialized_flag_;
  }

  /**
   * @brief GetOriginLongitude 获取初始经度
   * @return
   */
  inline double GetOriginLongitude() { return origin_longitude_; }

  /**
   * @brief GetOriginLatitude 获取初始纬度
   * @return
   */
  inline double GetOriginLatitude() { return origin_latitude_; }

  /**
   * @brief GetOriginAltitude 获取初始高度
   * @return
   */
  inline double GetOriginAltitude() { return origin_altitude_; }

 private:
  /**
   * @brief GNSSConverter 构造函数
   */
  GNSSConverter();

  /// @brief 实例
  static GNSSConverter *instance_;

 private:
  /// @brief 初始化位置标志位
  bool origin_position_initialized_flag_;

  /// @brief 坐标转换器
  GeographicLib::LocalCartesian geographic_converter_;

  /// @brief 初始经纬度
  double origin_longitude_;
  double origin_latitude_;
  double origin_altitude_;
};

}  // namespace multi_sensor_mapping

#endif
