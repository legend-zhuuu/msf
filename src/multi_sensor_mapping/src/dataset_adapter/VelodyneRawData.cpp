#include <angles/angles.h>
#include <dataset_adapter/VelodyneRawData.h>
#include <glog/logging.h>

namespace dataset_adapter {
inline float SQR(float val) { return val * val; }

VelodyneRawData::VelodyneRawData() {}

bool VelodyneRawData::LoadCalibration(std::string calibration_path) {
  calibration_.read(calibration_path);
  if (!calibration_.initialized) {
    LOG(INFO) << "Unable to open calibration file: " << calibration_path;
    return false;
  }
  return true;
}

void VelodyneRawData::SetParameters(std::string model, double min_range,
                                    double max_range, double view_direction,
                                    double view_width) {
  config_.model = model;
  config_.min_range = min_range;
  config_.max_range = max_range;

  // converting angle parameters into the velodyne reference (rad)
  config_.tmp_min_angle = view_direction + view_width / 2;
  config_.tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  config_.tmp_min_angle =
      fmod(fmod(config_.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  config_.tmp_max_angle =
      fmod(fmod(config_.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 perfomrs a centered double to int conversion
  config_.min_angle =
      100 * (2 * M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
  config_.max_angle =
      100 * (2 * M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }

  BuildTimings();
  SetupSinCosCache();
}

void VelodyneRawData::Unpack(const velodyne_msgs::VelodynePacket& pkt,
                             VelRTPointCloudPtr cloud,
                             const ros::Time& scan_start_time) {
  using velodyne_pointcloud::LaserCorrection;

  if (calibration_.num_lasers == 16) {
    /// TODO
    return;
  }

  float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();
  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[0];

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation

    int bank_origin = 0;
    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      const uint8_t laser_number = j + bank_origin;
      float time = 0;

      const LaserCorrection& corrections =
          calibration_.laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block_t& block = raw->blocks[i];
      union two_bytes tmp;
      tmp.bytes[0] = block.data[k];
      tmp.bytes[1] = block.data[k + 1];

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((block.rotation >= config_.min_angle &&
           block.rotation <= config_.max_angle &&
           config_.min_angle < config_.max_angle) ||
          (config_.min_angle > config_.max_angle &&
           (raw->blocks[i].rotation <= config_.max_angle ||
            raw->blocks[i].rotation >= config_.min_angle))) {
        if (timing_offsets.size()) {
          time = timing_offsets[i][j] + time_diff_start_to_this_packet;
        }

        if (tmp.uint == 0) {
          continue;
        }

        float distance = tmp.uint * calibration_.distance_resolution_m;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
            cos_rot_table_[block.rotation] * cos_rot_correction +
            sin_rot_table_[block.rotation] * sin_rot_correction;
        float sin_rot_angle =
            sin_rot_table_[block.rotation] * cos_rot_correction -
            cos_rot_table_[block.rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        float xy_distance =
            distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
        if (xx < 0) xx = -xx;
        if (yy < 0) yy = -yy;

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;
        if (corrections.two_pt_correction_available) {
          distance_corr_x =
              (corrections.dist_correction - corrections.dist_correction_x) *
                  (xx - 2.4) / (25.04 - 2.4) +
              corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y =
              (corrections.dist_correction - corrections.dist_correction_y) *
                  (yy - 1.93) / (25.04 - 1.93) +
              corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance =
            distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        /// the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance =
            distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k + 2];

        float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                             (1 - corrections.focal_distance / 13100);
        float focal_slope = corrections.focal_slope;
        intensity +=
            focal_slope *
            (std::abs(focal_offset -
                      256 * SQR(1 - static_cast<float>(tmp.uint) / 65535)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        VelRTPoint p;
        p.x = x_coord;
        p.y = y_coord;
        p.z = z_coord;
        p.intensity = intensity;
        p.ring = corrections.laser_ring;
        p.time = time;
        cloud->push_back(p);
      }
    }
  }
}

bool VelodyneRawData::BuildTimings() {
  // vlp16
  if (config_.model == "VLP16") {
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i = 0; i < timing_offsets.size(); ++i) {
      timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6;  // seconds
    double single_firing = 2.304 * 1e-6;       // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = (x - (x % 2)) + (y / 16);
        } else {
          dataBlockIndex = (x * 2) + (y / 16);
        }
        dataPointIndex = y % 16;
        // timing_offsets[block][firing]
        timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) +
                               (single_firing * dataPointIndex);
      }
    }
  }
  // vlp32
  else if (config_.model == "32C") {
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i = 0; i < timing_offsets.size(); ++i) {
      timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6;  // seconds
    double single_firing = 2.304 * 1e-6;       // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = x / 2;
        } else {
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) +
                               (single_firing * dataPointIndex);
      }
    }
  }
  // hdl32
  else if (config_.model == "32E") {
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i = 0; i < timing_offsets.size(); ++i) {
      timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 46.080 * 1e-6;  // seconds
    double single_firing = 1.152 * 1e-6;       // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = x / 2;
        } else {
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) +
                               (single_firing * dataPointIndex);
      }
    }
  } else {
    timing_offsets.clear();
    LOG(INFO) << "Timings not supported for model %s", config_.model.c_str();
    return false;
  }
  return true;
}

void VelodyneRawData::SetupSinCosCache() {
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
}

}  // namespace dataset_adapter
