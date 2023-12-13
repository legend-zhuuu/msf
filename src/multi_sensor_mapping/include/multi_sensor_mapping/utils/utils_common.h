#ifndef MSM_UTILS_COMMON_H
#define MSM_UTILS_COMMON_H

#include <Eigen/Eigen>
#include <chrono>
#include <fstream>
#include <memory>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

using Transf = Eigen::Matrix4f;
using TransfVec =
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >;

/// @brief 所有坐标系的类型
enum FrameType { IMU = 0, LIDAR, ODOM, BASELINK, GNSS };

/// @brief 建图模式
enum class MappingMode {
  LIDAR_IMU_MAPPING = 0,   // 激光-IMU融合建图
  LIDAR_ODOM_MAPPING = 1,  // 激光-轮速计融合建图
  PURE_LIDAR_MAPPING = 2,  // 纯激光建图
  CLINS_MAPPING = 3,       // CLINS建图
};

/// @brief 所有参数的类型
enum class ParamType : int {
  PARAM_TYPE_ERROR = 0,
  PARAM_TYPE_SENSOR,
  PARAM_TYPE_EXTRINSIC,
  PARAM_TYPE_LIDAR_IMU_MAPPING,
  PARAM_TYPE_PATCHWORK,
  PARAM_TYPE_CLINS,
  PARAM_TYPE_FAST_LIO,
  PARAM_TYPE_POSE_GRAPH,
  PARAM_TYPE_TINY_LOCATION,
  PARAM_TYPE_LIDAR_DETECTION
};

/// @brief 激光雷达数据格式
enum class LidarDataFormat {
  VAGUE = 0,
  VELODYNE = 1,                 // Velodyne
  RSLIDAR = 2,                  // rslidar 格式
  RSLIDAR_LEGACY = 3,           // rslidar(老版本) 格式
  LIVOX = 4,                    // Livox 数据格式
  OUSTER = 5,                   // Ouster 数据格式
  SURESTAR = 6,                 // 北科天绘 数据格式
  HESAI = 7,                    // Hesai 数据格式
  RS_PACKET_RSHELIOS_16P = 10,  // RS16 packet 数据格式
  RS_PACKET_RSHELIOS = 11,      // RS32 packet 数据格式
  RS_PACKET_80 = 12             // RS80 packet 数据格式
};

/**
 * @brief The SensorPurpose enum 传感器用途
 *
 */
enum class SensorPurpose {
  SP_LIO = 0,  // LIO在线建图
  SP_LIM       // LIM离线建图
};

/**
 * @brief The LidarName enum 激光名字
 */
enum class LidarName {
  LIDAR_A = 0,
  LIDAR_B = 1,
  LIDAR_C = 2,
  LIDAR_D = 3,
  LIDAR_E = 4
};

/// @brief 标志位状态
enum class FlagStatus {
  FLAG_NOT_UPDATED = 0,  /// 标志位未更新
  FLAG_UPDATING,         /// 标志位等待更新
  FLAG_UPDATE_SUCCESS,   /// 标志位更新成功
  FLAG_UPDATE_FAIL       /// 标志位更新失败

};

enum class MotionState {
  STATE_UNKNOWN = 0,  /// 状态未知
  STATE_STATIC,       /// 状态静止
  STATE_DYNAMIC       /// 状态运动
};

/**
 * @brief 同步状态
 *
 */
enum class SyncState {
  SYNC_FAIL = 0,     // 同步失败
  SYNC_SUCCESS = 1,  // 同步成功
  SYNC_WAITING = 2,  // 同步等待
  SYNC_TIMEOUT = 3   // 同步超时
};

/**
 * @brief 处理状态
 *
 */
enum class ProcessState {
  PROCESS_START = 0,  // 开始处理
  PROCESS_FINSIH      // 结束处理
};

/**
 * @brief The MatchMethod enum 定义配准算法
 */
enum class MatchMethod {
  MATCH_METHOD_LOAM = 0,   // LOAM
  MATCH_METHOD_FAST_LOAM,  // Fast-LOAM
  MATCH_METHOD_FAST_LIO    // Fast-LIO
};

/// @brief 默认输出文件夹
static const std::string OUTPUT_FOLDER_DIR = "~/.multi_sensor_mapping";

inline bool IsFileExist(std::string _file_name) {
  std::ifstream f(_file_name.c_str());
  return f.good();
}

inline LidarDataFormat StrToLidarDataFormat(const std::string _type) {
  if (_type == "RSLIDAR") {
    return LidarDataFormat::RSLIDAR;
  } else if (_type == "VELODYNE") {
    return LidarDataFormat::VELODYNE;
  } else if (_type == "RSLIDAR_LEGACY") {
    return LidarDataFormat::RSLIDAR_LEGACY;
  } else if (_type == "LIVOX") {
    return LidarDataFormat::LIVOX;
  } else if (_type == "OUSTER") {
    return LidarDataFormat::OUSTER;
  } else if (_type == "SURESTAR") {
    return LidarDataFormat::SURESTAR;
  } else if (_type == "HESAI") {
    return LidarDataFormat::HESAI;
  } else if (_type == "RS_PACKET_RSHELIOS_16P") {
    return LidarDataFormat::RS_PACKET_RSHELIOS_16P;
  } else if (_type == "RS_PACKET_RSHELIOS") {
    return LidarDataFormat::RS_PACKET_RSHELIOS;
  }

  return LidarDataFormat::VAGUE;
}

inline LidarName StrToLidarName(const std::string _name) {
  if (_name == "A") {
    return LidarName::LIDAR_A;
  } else if (_name == "B") {
    return LidarName::LIDAR_B;
  } else if (_name == "C") {
    return LidarName::LIDAR_C;
  } else if (_name == "D") {
    return LidarName::LIDAR_D;
  }
  return LidarName::LIDAR_E;
}

inline MatchMethod StrToMatchMethod(const std::string _method) {
  if (_method == "LOAM") {
    return MatchMethod::MATCH_METHOD_LOAM;
  } else if (_method == "FAST_LOAM") {
    return MatchMethod::MATCH_METHOD_FAST_LOAM;
  } else if (_method == "FAST_LIO") {
    return MatchMethod::MATCH_METHOD_FAST_LIO;
  }
  return MatchMethod::MATCH_METHOD_FAST_LOAM;
}

inline std::string LidarDataFormatToStr(const LidarDataFormat& _type) {
  std::string str = "";
  switch (_type) {
    case LidarDataFormat::RSLIDAR:
      str = "Robosense-LiDAR";
      break;
    case LidarDataFormat::RSLIDAR_LEGACY:
      str = "Robosense-LiDAR(Legacy)";
      break;
    case LidarDataFormat::VELODYNE:
      str = "Velodyne-LiDAR";
      break;
    case LidarDataFormat::LIVOX:
      str = "Livox-LiDAR";
      break;
    case LidarDataFormat::OUSTER:
      str = "Ouster-LiDAR";
      break;
    case LidarDataFormat::SURESTAR:
      str = "SureStar-LiDAR";
      break;
    case LidarDataFormat::HESAI:
      str = "Hesai-LiDAR";
      break;
    case LidarDataFormat::RS_PACKET_RSHELIOS_16P:
      str = "Robosense-RSHELIOS-16P";
      break;
    case LidarDataFormat::RS_PACKET_RSHELIOS:
      str = "Robosense-RSHELIOS";
      break;
    default:
      str = "ERROR";
      break;
  }

  return str;
}

inline std::string LidarNameToStr(const LidarName& _name) {
  std::string str = "";
  switch (_name) {
    case LidarName::LIDAR_A:
      str = "LiDAR-A";
      break;
    case LidarName::LIDAR_B:
      str = "LiDAR-B";
      break;
    case LidarName::LIDAR_C:
      str = "LiDAR-C";
      break;
    case LidarName::LIDAR_D:
      str = "LiDAR-D";
      break;
    case LidarName::LIDAR_E:
      str = "LiDAR-E";
      break;
    default:
      str = "ERROR";
      break;
  }

  return str;
}

inline std::string MatchMethodToStr(const MatchMethod& _method) {
  std::string str = "";
  switch (_method) {
    case MatchMethod::MATCH_METHOD_LOAM:
      str = "LOAM";
      break;
    case MatchMethod::MATCH_METHOD_FAST_LOAM:
      str = "FAST_LAOM";
      break;
    default:
      str = "ERROR";
      break;
  }

  return str;
}

inline SensorPurpose StrToSensorPurpose(const std::string _purpose) {
  if (_purpose == "LIO") {
    return SensorPurpose::SP_LIO;
  } else if (_purpose == "LIM") {
    return SensorPurpose::SP_LIM;
  }
  return SensorPurpose::SP_LIM;
}

inline std::string SensorPurposeToStr(const SensorPurpose& _purpose) {
  std::string str = "";
  switch (_purpose) {
    case SensorPurpose::SP_LIO:
      str = "LIO-ONLINE";
      break;
    case SensorPurpose::SP_LIM:
      str = "LIM-OFFLINE";
      break;
    default:
      str = "ERROR";
      break;
  }

  return str;
}

/**
 * @brief  获取系统时间
 *
 * @return double
 */
inline double GetSystemTimeSecond() {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>
      current_time = std::chrono::time_point_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now());
  return (double)current_time.time_since_epoch().count() * 1e-3;
}

#endif
