
#ifndef MSM_UTILS_POINTCLOUD_H
#define MSM_UTILS_POINTCLOUD_H

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include "multi_sensor_mapping/utils/utils_common.h"

/**
 * @brief The PointXYZIRT struct velodyne的数据格式
 */
struct PointXYZIRT {
  PCL_ADD_POINT4D;                 ///< quad-word XYZ
  float intensity;                 ///< laser intensity reading
  std::uint16_t ring;              ///< laser ring number
  float time;                      ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(std::uint16_t, ring,
                                                       ring)(float, time, time))
/**
 * @brief The RsPointXYZIRT struct 速腾激光雷达数据格式(v1.5.8版本后)
 */
struct RsPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint16_t, ring, ring)(double, timestamp, timestamp))

/**
 * @brief The LegacyRsPointXYZIRT struct 速腾激光雷达数据格式(v1.5.8版本之前)
 */
struct LegacyRsPointXYZIRT {
  PCL_ADD_POINT4D;
  std::uint8_t intensity;
  std::uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LegacyRsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
        std::uint16_t, ring, ring)(double, timestamp, timestamp))

/**
 * @brief The OsPointXYZIRT struct Ouester数据格式
 */
struct OsPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  std::uint8_t ring;  // The channel index
  std::uint16_t noise;
  std::uint32_t range;  // The distance measurement
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    OsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(
        std::uint8_t, ring, ring)(std::uint16_t, noise, noise)(std::uint32_t,
                                                               range, range))

/**
 * @brief The SureStarPointXYZIRT struct Rfans数据格式(北科天绘)
 */
struct SureStarPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;   // 激光强度
  float v_angle;     // 转台角度
  float h_angle;     // 水平角
  float range;       // 距离
  double timestamp;  // 时间戳
  int laserid;       // 线号
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    SureStarPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, v_angle, v_angle)(float, h_angle, h_angle)(float, range, range)(
        double, timestamp, timestamp)(int, laserid, laserid))

/**
 * @brief The HesaiPointXYZIRT struct Hesai 数据格式
 *
 */
struct HesaiPointXYZIRT {
  PCL_ADD_POINT4D
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    HesaiPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

/**
 * @brief The LivoxPointXYZIRT struct Livox数据格式(MID360)
 *
 */
struct LivoxPointXYZIRT {
  PCL_ADD_POINT4D
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))

/**
 * @brief 自定义点云数据格式
 *
 */
struct KPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring = 0;
  std::uint32_t timestamp_ns = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    KPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint16_t, ring, ring)(std::uint32_t, timestamp_ns, timestamp_ns))

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef CloudType::Ptr CloudTypePtr;

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef ColorPointCloud::Ptr ColorPointCloudPtr;

typedef PointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;
typedef RTPointCloud::Ptr RTPointCloudPtr;

typedef RsPointXYZIRT RsRTPoint;
typedef pcl::PointCloud<RsRTPoint> RsRTPointCloud;
typedef RsRTPointCloud::Ptr RsRTPointCloudPtr;

typedef LegacyRsPointXYZIRT LegacyRsRTPoint;
typedef pcl::PointCloud<LegacyRsRTPoint> LegacyRsRTPointCloud;
typedef LegacyRsRTPointCloud::Ptr LegacyRsRTPointCloudPtr;

typedef PointXYZIRT VelRTPoint;
typedef pcl::PointCloud<VelRTPoint> VelRTPointCloud;
typedef VelRTPointCloud::Ptr VelRTPointCloudPtr;

typedef OsPointXYZIRT OsRTPoint;
typedef pcl::PointCloud<OsRTPoint> OslRTPointCloud;
typedef OslRTPointCloud::Ptr OsRTPointCloudPtr;

typedef SureStarPointXYZIRT SsRTPoint;
typedef pcl::PointCloud<SsRTPoint> SslRTPointCloud;
typedef SslRTPointCloud::Ptr SsRTPointCloudPtr;

typedef HesaiPointXYZIRT HsRTPoint;
typedef pcl::PointCloud<HsRTPoint> HsRTPointCloud;
typedef HsRTPointCloud::Ptr HsRTPointCloudPtr;

typedef LivoxPointXYZIRT LvRTPoint;
typedef pcl::PointCloud<LvRTPoint> LvRTPointCloud;
typedef LvRTPointCloud::Ptr LvRTPointCloudPtr;

typedef KPointXYZIRT KRTPoint;
typedef pcl::PointCloud<KRTPoint> KRTPointCloud;
typedef KRTPointCloud::Ptr KRTPointCloudPtr;

template <typename T_Point>
class PointCloudT : public pcl::PointCloud<T_Point> {
 public:
  typedef T_Point PointT;
  typedef typename pcl::PointCloud<T_Point>::VectorType VectorT;

  double timestamp = 0.0;
  uint32_t seq = 0;
};

typedef PointCloudT<RsRTPoint> RsPointCloudMsg;

template <typename PT>
float PointDistance(PT point) {
  return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

template <typename PT>
float PointDistance(PT p1, PT p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

template <typename PT>
float PointDistanceSquare(PT point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}

template <typename PT>
float PointDistanceXY(PT point) {
  return sqrt(point.x * point.x + point.y * point.y);
}

namespace multi_sensor_mapping {

namespace utils {

template <typename PointT>
void DownsampleCloud(pcl::PointCloud<PointT>& _cloud, double _leaf_size) {
  pcl::VoxelGrid<PointT> sor;
  typename pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
  *cloud_in = _cloud;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
  sor.filter(_cloud);
}

template <typename PointT>
void UniformSampleCloud(pcl::PointCloud<PointT>& _cloud_in,
                        pcl::PointCloud<PointT>& _cloud_out,
                        double _leaf_size) {
  _cloud_out.clear();
  pcl::UniformSampling<PointT> uniform_sampling;
  typename pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
  *cloud_in = _cloud_in;
  uniform_sampling.setRadiusSearch(_leaf_size);
  uniform_sampling.setInputCloud(cloud_in);
  uniform_sampling.filter(_cloud_out);
}

template <typename PointT>
void DownsampleCloudAdapted(pcl::PointCloud<PointT>& cloud_in,
                            pcl::PointCloud<PointT>& cloud_out,
                            double leaf_size) {
  cloud_out.clear();

  Eigen::Vector4f min_p;  //用于存放三个轴的最小值
  Eigen::Vector4f max_p;  //用于存放三个轴的最大值
  pcl::getMinMax3D(cloud_in, min_p, max_p);

  std::int64_t dx, dy, dz;

  double temp_dx = (max_p[0] - min_p[0]) / leaf_size;
  double temp_dy = (max_p[1] - min_p[1]) / leaf_size;
  double temp_dz = (max_p[2] - min_p[2]) / leaf_size;

  /// 自动切分
  Eigen::Vector3d splite_num(1, 1, 1);
  while (true) {
    dx = static_cast<std::int64_t>(temp_dx / splite_num(0)) + 1;
    dy = static_cast<std::int64_t>(temp_dy / splite_num(1)) + 1;
    dz = static_cast<std::int64_t>(temp_dz / splite_num(2)) + 1;

    if ((dx * dy * dz) <
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
      break;
    }

    if (dx > dy) {
      splite_num(0) += 1;
    } else {
      splite_num(1) += 1;
    }
  }

  //  std::cout << "Splite num : " << splite_num.transpose() << std::endl;

  double stepX = (max_p[0] - min_p[0]) / splite_num(0);
  double stepY = (max_p[1] - min_p[1]) / splite_num(1);

  for (int i = 0; i < splite_num(0); i++) {
    for (int j = 0; j < splite_num(1); j++) {
      // 获得子区域中的点云
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud_in.makeShared());
      pass.setFilterFieldName("x");
      pass.setFilterLimits(
          min_p[0] + i * stepX,
          min_p[0] + i * stepX + stepX);  //保留或过滤z轴方向-1.2到0
      // pass.setFilterLimitsNegative(true);//设置过滤器限制负//设置保留范围内false
      pcl::PointCloud<PointT> cloud_filtered;
      pass.filter(cloud_filtered);

      pass.setInputCloud(cloud_filtered.makeShared());
      pass.setFilterFieldName("y");
      pass.setFilterLimits(
          min_p[1] + j * stepY,
          min_p[1] + j * stepY + stepY);  //保留或过滤z轴方向-1.2到0
                                          // VPointCloud cloud_filtered;
      pass.filter(cloud_filtered);

      pcl::VoxelGrid<PointT> sor;
      sor.setInputCloud(cloud_filtered.makeShared());
      sor.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
      pcl::PointCloud<PointT> cloud_downsample;
      sor.filter(cloud_downsample);

      cloud_out += cloud_downsample;
    }
  }
}

template <typename TPoint>
float PointDistance(const TPoint& p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename TPoint>
float PointRadius(const TPoint& p) {
  return sqrt(p.x * p.x + p.y * p.y);
}

template <typename TPoint>
float PointDistance(const TPoint& p1, const TPoint& p2) {
  double x = p1.x - p2.x;
  double y = p1.y - p2.y;
  double z = p1.z - p2.z;
  return sqrt(x * x + y * y + z * z);
}

}  // namespace utils

}  // namespace multi_sensor_mapping

#endif
