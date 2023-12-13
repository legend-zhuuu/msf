#ifndef MSM_RSLIDAR_DECODER_H
#define MSM_RSLIDAR_DECODER_H

#include <rs_driver/api/lidar_driver.hpp>

#include "multi_sensor_mapping/utils/rslidar_msg/rslidar_packet.hpp"
#include "multi_sensor_mapping/utils/utils_common.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief The RslidarDecoder class RsLidar packet数据解析器
 */
class RslidarDecoder {
 public:
  typedef std::shared_ptr<RslidarDecoder> Ptr;

  /**
   * @brief RslidarDecoder 构造函数
   * @param lidar_type
   */
  RslidarDecoder(std::string _lidar_type = "RSHELIOS");

  /**
   * @brief RslidarDecoder 构造函数
   *
   * @param _format
   */
  RslidarDecoder(LidarDataFormat _format = LidarDataFormat::RS_PACKET_RSHELIOS);

  /**
   * @brief 解析Packet (v1.5.x)
   *
   * @param msg
   * @return bool
   */
  bool DecodePacket(const rslidar_msg::RslidarPacket& msg);

  /**
   * @brief Get the Cloud Queue Size object
   *
   * @return size_t
   */
  size_t GetCloudQueueSize();

  /**
   * @brief Get the Cloud object 获取解析的点云数据
   *
   * @return std::shared_ptr<RsPointCloudMsg>
   */
  std::shared_ptr<RsPointCloudMsg> GetCloud();

 private:
  /**
   * @brief 格式转换
   *
   * @param ros_msg
   * @return robosense::lidar::Packet
   */
  robosense::lidar::Packet ConvertRsMsg(
      const rslidar_msg::RslidarPacket& ros_msg);

  /**
   * @brief 分离帧 (decoder中的调用的回调函数)
   *
   * @param height
   * @param ts
   */
  void SplitFrame(uint16_t height, double ts);

  /**
   * @brief 异常回调 (decoder中的调用的回调函数)
   *
   * @param error
   */
  void ExceptionCallback(const robosense::lidar::Error& error);

 private:
  /// @brief 激光类型
  robosense::lidar::LidarType lidar_type_;
  /// @brief 解析参数
  robosense::lidar::RSDecoderParam decode_param_;

  /// @brief Lidar数据解析器
  std::shared_ptr<robosense::lidar::Decoder<RsPointCloudMsg>>
      lidar_decoder_ptr_;
  /// @brief 点云队列
  std::queue<std::shared_ptr<RsPointCloudMsg>> stuffed_cloud_queue_;
};

}  // namespace multi_sensor_mapping
#endif
