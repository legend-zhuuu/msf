#include "multi_sensor_mapping/utils/rslidar_decoder.h"

namespace multi_sensor_mapping {

using namespace robosense::lidar;

RslidarDecoder::RslidarDecoder(std::string _lidar_type) {
  lidar_type_ = robosense::lidar::strToLidarType(_lidar_type);
  decode_param_.use_lidar_clock = true;
  lidar_decoder_ptr_ = DecoderFactory<RsPointCloudMsg>::createDecoder(
      lidar_type_, decode_param_);
  // lidar_decoder_ptr_->enableWritePktTs(false);

  lidar_decoder_ptr_->point_cloud_ = std::make_shared<RsPointCloudMsg>();
  lidar_decoder_ptr_->regCallback(
      std::bind(&RslidarDecoder::ExceptionCallback, this,
                std::placeholders::_1),
      std::bind(&RslidarDecoder::SplitFrame, this, std::placeholders::_1,
                std::placeholders::_2));
}

RslidarDecoder::RslidarDecoder(LidarDataFormat _format) {
  std::string lidar_type_str;
  switch (_format) {
    case LidarDataFormat::RS_PACKET_RSHELIOS_16P:
      lidar_type_str = "RSHELIOS_16P";
      break;
    case LidarDataFormat::RS_PACKET_RSHELIOS:
      lidar_type_str = "RSHELIOS";
      break;
    default:
      lidar_type_str = "";
      break;
  }

  lidar_type_ = robosense::lidar::strToLidarType(lidar_type_str);
  decode_param_.use_lidar_clock = true;
  lidar_decoder_ptr_ = DecoderFactory<RsPointCloudMsg>::createDecoder(
      lidar_type_, decode_param_);
  // lidar_decoder_ptr_->enableWritePktTs(false);

  lidar_decoder_ptr_->point_cloud_ = std::make_shared<RsPointCloudMsg>();
  lidar_decoder_ptr_->regCallback(
      std::bind(&RslidarDecoder::ExceptionCallback, this,
                std::placeholders::_1),
      std::bind(&RslidarDecoder::SplitFrame, this, std::placeholders::_1,
                std::placeholders::_2));
}

bool RslidarDecoder::DecodePacket(const rslidar_msg::RslidarPacket& msg) {
  bool res = false;
  Packet packet = ConvertRsMsg(msg);

  std::shared_ptr<Buffer> pkt = std::make_shared<Buffer>(ETH_LEN);
  memcpy(pkt->data(), packet.buf_.data(), packet.buf_.size());
  pkt->setData(0, packet.buf_.size());

  uint8_t* id = pkt->data();
  if (*id == 0x55) {
    res = lidar_decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());

  } else if (*id == 0xA5) {
    lidar_decoder_ptr_->processDifopPkt(pkt->data(), pkt->dataSize());
  }

  return res;
}

size_t RslidarDecoder::GetCloudQueueSize() {
  return stuffed_cloud_queue_.size();
}

std::shared_ptr<RsPointCloudMsg> RslidarDecoder::GetCloud() {
  if (!stuffed_cloud_queue_.empty()) {
    std::shared_ptr<RsPointCloudMsg> msg = stuffed_cloud_queue_.front();
    stuffed_cloud_queue_.pop();
    return msg;
  }
  return std::make_shared<RsPointCloudMsg>();
}

Packet RslidarDecoder::ConvertRsMsg(const rslidar_msg::RslidarPacket& ros_msg) {
  Packet rs_msg;
  rs_msg.timestamp = ros_msg.header.stamp.toSec();
  rs_msg.seq = ros_msg.header.seq;
  rs_msg.is_difop = ros_msg.is_difop;
  rs_msg.is_frame_begin = ros_msg.is_frame_begin;

  for (size_t i = 0; i < ros_msg.data.size(); i++) {
    rs_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return rs_msg;
}

void RslidarDecoder::SplitFrame(uint16_t height, double ts) {
  std::shared_ptr<RsPointCloudMsg> cloud = lidar_decoder_ptr_->point_cloud_;
  if (cloud->points.size() > 0) {
    // 设置点云基础信息
    cloud->timestamp = ts;
    cloud->is_dense = decode_param_.dense_points;
    if (cloud->is_dense) {
      cloud->height = 1;
      cloud->width = (uint32_t)cloud->points.size();
    } else {
      cloud->height = height;
      cloud->width = (uint32_t)cloud->points.size() / cloud->height;
    }

    stuffed_cloud_queue_.push(cloud);
    lidar_decoder_ptr_->point_cloud_ = std::make_shared<RsPointCloudMsg>();
  }
}

void RslidarDecoder::ExceptionCallback(const Error& error) {
  /// TODO
}

}  // namespace multi_sensor_mapping
