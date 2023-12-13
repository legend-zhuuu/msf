/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef VELODYNE_RAW_DATA_H
#define VELODYNE_RAW_DATA_H

#include <multi_sensor_localization/utility.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/calibration.h>

namespace dataset_adapter {

/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;        // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;    // [µs]

typedef struct raw_block {
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

typedef struct raw_packet {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

/**
 * @brief The VelodyneRawData class
 * Adapted from velodyne
 */
class VelodyneRawData {
 public:
  VelodyneRawData();

  /**
   * @brief LoadCalibration 加载配置文件
   * @param calibration_path
   * @return
   */
  bool LoadCalibration(std::string calibration_path);

  /**
   * @brief SetParameters 设置参数
   * @param model
   * @param min_range
   * @param max_range
   * @param view_direction
   * @param view_width
   */
  void SetParameters(std::string model, double min_range, double max_range,
                     double view_direction, double view_width);

  /**
   * @brief Unpack 数据解析
   * @param pkt
   * @param cloud
   * @param scan_start_time
   */
  void Unpack(const velodyne_msgs::VelodynePacket& pkt,
              VelRTPointCloudPtr cloud, const ros::Time& scan_start_time);

 private:
  /**
   * @brief BuildTimings 构造时间序列
   * @return
   */
  bool BuildTimings();

  /**
   * @brief SetupSinCosCache
   */
  void SetupSinCosCache();

 private:
  /** configuration parameters */
  typedef struct {
    std::string model;
    std::string calibrationFile;  ///< calibration file name
    double max_range;             ///< maximum range to publish
    double min_range;             ///< minimum range to publish
    int min_angle;                ///< minimum angle to publish
    int max_angle;                ///< maximum angle to publish

    double tmp_min_angle;
    double tmp_max_angle;
  } Config;
  Config config_;

  /**
   * Calibration file
   */
  velodyne_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // timing offset lookup table
  std::vector<std::vector<float> > timing_offsets;
};

}  // namespace dataset_adapter

#endif
