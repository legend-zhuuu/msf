#ifndef MSM_UTILS_GEOMETRY_H
#define MSM_UTILS_GEOMETRY_H

#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

namespace multi_sensor_mapping {

namespace utils {

/**
 * @brief GetTranslationAndEulerAngles 获取pose的坐标与欧拉角
 * @param _t
 * @param _x
 * @param _y
 * @param _z
 * @param _roll
 * @param _pitch
 * @param _yaw
 */
void GetTranslationAndEulerAngles(const Eigen::Matrix4d& _t, double& _x,
                                  double& _y, double& _z, double& _roll,
                                  double& _pitch, double& _yaw);
/**
 * @brief GetDistanceBetweenPoses 获取两帧pose间的欧氏距离
 * @param _pose1
 * @param _pose2
 * @return
 */
double GetDistanceBetweenPoses(const Eigen::Matrix4d& _pose1,
                               const Eigen::Matrix4d& _pose2);

/**
 * @brief TransfToTransform eigen的pose转换为tf的transform
 * @param _trans
 * @return
 */
tf::Transform TransfToTransform(const Eigen::Matrix4f& _trans);

/**
 * @brief TransfToTransform eigen的pose转换为tf的transform
 * @param _trans
 * @return
 */
tf::Transform TransdToTransform(const Eigen::Matrix4d& _trans);

/**
 * @brief XYZYPR2Transf XYZYPR转旋转矩阵
 * @param x
 * @param y
 * @param z
 * @param yaw
 * @param pitch
 * @param roll
 * @return
 */
Eigen::Matrix4d XYZYPR2Transd(double x, double y, double z, double yaw,
                              double pitch, double roll);

/**
 * @brief 旋转矩阵转欧拉角
 *
 * @param mat
 * @param yaw
 * @param pitch
 * @param roll
 */
void RoationMatrixd2YPR(Eigen::Matrix3d mat, double& yaw, double& pitch,
                        double& roll);

/**
 * @brief 变换矩阵转XYZYPR
 *
 * @param mat
 * @param x
 * @param y
 * @param z
 * @param yaw
 * @param pitch
 * @param roll
 */
void Transd2XYZYPR(Eigen::Matrix4d mat, double& x, double& y, double& z,
                   double& yaw, double& pitch, double& roll);

/**
 * @brief SlerpPose 对pose进行插值(旋转部分为球面线性插值)
 * @param _pose_front
 * @param _pose_back
 * @param _ratio 计算方式为_ratio = (t_out - t_back) / (t_front - t_back)
 * @return
 */
Eigen::Matrix4f SlerpPose(const Eigen::Matrix4f& _pose_front,
                          const Eigen::Matrix4f& _pose_back,
                          const double& _ratio);

/**
 * @brief 旋转矩阵转欧拉角
 *
 * @param _mat
 * @return Eigen::Vector3d
 */
Eigen::Vector3d R2ypr(const Eigen::Matrix3d& _mat);

/**
 * @brief GetTransformFromSe3 从Se3中获取q和t
 * @param se3
 * @param q
 * @param t
 */
void GetTransformFromSe3(const Eigen::Matrix<double, 6, 1>& se3,
                         Eigen::Quaterniond& q, Eigen::Vector3d& t);

inline std::vector<double> ToVector(const Eigen::Vector3d& _vec) {
  return std::vector<double>(&_vec(0), _vec.data() + _vec.cols() * _vec.rows());
}

inline std::vector<double> ToVector(const Eigen::Vector4d& _vec) {
  return std::vector<double>(&_vec(0), _vec.data() + _vec.cols() * _vec.rows());
}

inline std::vector<double> ToVector(const Eigen::Quaterniond& _quater) {
  return ToVector(_quater.coeffs());
}

/**
 * @brief 平移旋转 --> 矩阵
 *
 * @param _pos
 * @param _quater
 * @return Eigen::Matrix4d
 */
Eigen::Matrix4d ToMatrix4d(const Eigen::Vector3d& _pos,
                           const Eigen::Quaterniond& _quater);

/**
 * @brief 旋转点
 *
 * @param _point
 * @param _mat
 * @return Eigen::Vector3d
 */
Eigen::Vector3d TransformPoint(const Eigen::Vector3d& _point,
                               const Eigen::Matrix4d& _mat);

}  // namespace utils

}  // namespace multi_sensor_mapping

#endif
