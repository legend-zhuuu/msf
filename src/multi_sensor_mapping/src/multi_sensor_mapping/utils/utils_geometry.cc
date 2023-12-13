#include "multi_sensor_mapping/utils/utils_geometry.h"

#include <pcl/common/eigen.h>

#include "multi_sensor_mapping/utils/utils_eigen.h"

namespace multi_sensor_mapping {

namespace utils {

void GetTranslationAndEulerAngles(const Eigen::Matrix4d& _t, double& _x,
                                  double& _y, double& _z, double& _roll,
                                  double& _pitch, double& _yaw) {
  Eigen::Affine3d transAffine;
  transAffine.matrix() = _t;
  pcl::getTranslationAndEulerAngles(transAffine, _x, _y, _z, _roll, _pitch,
                                    _yaw);
}

double GetDistanceBetweenPoses(const Eigen::Matrix4d& _pose1,
                               const Eigen::Matrix4d& _pose2) {
  double x(_pose1(0, 3) - _pose2(0, 3));
  double y(_pose1(1, 3) - _pose2(1, 3));
  double z(_pose1(2, 3) - _pose2(2, 3));
  return sqrt(x * x + y * y + z * z);
}

tf::Transform TransfToTransform(const Eigen::Matrix4f& _trans) {
  tf::Transform t;
  t.setOrigin(tf::Vector3(_trans(0, 3), _trans(1, 3), _trans(2, 3)));

  Eigen::Quaternionf q(_trans.block<3, 3>(0, 0));
  t.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

  return t;
}

tf::Transform TransdToTransform(const Eigen::Matrix4d& _trans) {
  tf::Transform t;
  t.setOrigin(tf::Vector3(_trans(0, 3), _trans(1, 3), _trans(2, 3)));

  Eigen::Quaterniond q(_trans.block<3, 3>(0, 0));
  t.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

  return t;
}

Eigen::Matrix4d XYZYPR2Transd(double x, double y, double z, double yaw,
                              double pitch, double roll) {
  Eigen::Matrix4d result_matrix = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond quater(
      Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)) *
      Eigen::AngleAxisd(pitch * M_PI / 180.0, Eigen::Vector3d(0, 1, 0)) *
      Eigen::AngleAxisd(roll * M_PI / 180.0, Eigen::Vector3d(1, 0, 0)));
  result_matrix.block<3, 3>(0, 0) = quater.toRotationMatrix();
  result_matrix(0, 3) = x;
  result_matrix(1, 3) = y;
  result_matrix(2, 3) = z;

  return result_matrix;
}

void RoationMatrixd2YPR(Eigen::Matrix3d mat, double& yaw, double& pitch,
                        double& roll) {
  Eigen::Vector3d n = mat.col(0);
  Eigen::Vector3d o = mat.col(1);
  Eigen::Vector3d a = mat.col(2);

  yaw = atan2(n(1), n(0));
  pitch = atan2(-n(2), n(0) * cos(yaw) + n(1) * sin(yaw));
  roll = atan2(a(0) * sin(yaw) - a(1) * cos(yaw),
               -o(0) * sin(yaw) + o(1) * cos(yaw));

  // heading 0~2PI
  if (yaw < 0) {
    yaw = M_PI * 2 + yaw;
  }
}

void Transd2XYZYPR(Eigen::Matrix4d mat, double& x, double& y, double& z,
                   double& yaw, double& pitch, double& roll) {
  x = mat(0, 3);
  y = mat(1, 3);
  z = mat(2, 3);

  Eigen::Matrix3d dcm = mat.block<3, 3>(0, 0);

  RoationMatrixd2YPR(dcm, yaw, pitch, roll);

  yaw *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  roll *= 180.0 / M_PI;
}

Eigen::Matrix4f SlerpPose(const Eigen::Matrix4f& _pose_front,
                          const Eigen::Matrix4f& _pose_back,
                          const double& _ratio) {
  Eigen::Quaternionf q_front(_pose_front.block<3, 3>(0, 0));
  Eigen::Quaternionf q_back(_pose_back.block<3, 3>(0, 0));
  Eigen::Quaternionf q_out = q_back.slerp(_ratio, q_front);

  double x_out = _pose_back(0, 3) * (1 - _ratio) + _pose_front(0, 3) * _ratio;
  double y_out = _pose_back(1, 3) * (1 - _ratio) + _pose_front(1, 3) * _ratio;
  double z_out = _pose_back(2, 3) * (1 - _ratio) + _pose_front(2, 3) * _ratio;

  Eigen::Matrix4f result;
  result.block<3, 3>(0, 0) = q_out.toRotationMatrix();
  result(0, 3) = x_out;
  result(1, 3) = y_out;
  result(2, 3) = z_out;
  return result;
}

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& _mat) {
  Eigen::Vector3d n = _mat.col(0);
  Eigen::Vector3d o = _mat.col(1);
  Eigen::Vector3d a = _mat.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
  double r = atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
                   -o(0) * std::sin(y) + o(1) * std::cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;
}

void GetTransformFromSe3(const Eigen::Matrix<double, 6, 1>& se3,
                         Eigen::Quaterniond& q, Eigen::Vector3d& t) {
  Eigen::Vector3d omega(se3.data());
  Eigen::Vector3d upsilon(se3.data() + 3);
  Eigen::Matrix3d Omega = Eigen::SkewSymmetric(omega);

  double theta = omega.norm();
  double half_theta = 0.5 * theta;

  double imag_factor;
  double real_factor = cos(half_theta);
  if (theta < 1e-10) {
    double theta_sq = theta * theta;
    double theta_po4 = theta_sq * theta_sq;
    imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
  }

  q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(),
                         imag_factor * omega.y(), imag_factor * omega.z());

  Eigen::Matrix3d J;
  if (theta < 1e-10) {
    J = q.matrix();
  } else {
    Eigen::Matrix3d Omega2 = Omega * Omega;
    J = (Eigen::Matrix3d::Identity() +
         (1 - cos(theta)) / (theta * theta) * Omega +
         (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
  }

  t = J * upsilon;
}

Eigen::Matrix4d ToMatrix4d(const Eigen::Vector3d& _pos,
                           const Eigen::Quaterniond& _quater) {
  Eigen::Matrix4d result_mat = Eigen::Matrix4d::Identity();
  result_mat.block<3, 3>(0, 0) = _quater.toRotationMatrix();
  result_mat.block<3, 1>(0, 3) = _pos;
  return result_mat;
}
Eigen::Vector3d TransformPoint(const Eigen::Vector3d& _point,
                               const Eigen::Matrix4d& _mat) {
  Eigen::Vector3d transformed_point;
  transformed_point(0) = _mat(0, 0) * _point(0) + _mat(0, 1) * _point(1) +
                         _mat(0, 2) * _point(2) + _mat(0, 3);
  transformed_point(1) = _mat(1, 0) * _point(0) + _mat(1, 1) * _point(1) +
                         _mat(1, 2) * _point(2) + _mat(1, 3);
  transformed_point(2) = _mat(2, 0) * _point(0) + _mat(2, 1) * _point(1) +
                         _mat(2, 2) * _point(2) + _mat(2, 3);
  return transformed_point;
}
}  // namespace utils

}  // namespace multi_sensor_mapping
