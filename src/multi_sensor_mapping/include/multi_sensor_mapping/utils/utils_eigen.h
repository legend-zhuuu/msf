#ifndef MSM_UTILS_EIGEN_H
#define MSM_UTILS_EIGEN_H

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

namespace Eigen {
template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

template <typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>,
                             Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename K, typename V>
using aligned_unordered_map =
    std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                       Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> positify(
    const Eigen::QuaternionBase<Derived>& q) {
  // printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
  // Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(),
  // -q.z()); printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z()); return
  // q.template w() >= (typename Derived::Scalar)(0.0) ? q :
  // Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(),
  // -q.z());
  return q;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
    const Eigen::MatrixBase<Derived>& q) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
      typename Derived::Scalar(0), -q(0), -q(1), q(0),
      typename Derived::Scalar(0);
  return ans;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(
    const Eigen::QuaternionBase<Derived>& q) {
  Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
  ans.template block<3, 1>(
      1, 0) = qq.vec(),
         ans.template block<3, 3>(1, 1) =
             qq.w() *
                 Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
             SkewSymmetric(qq.vec());
  return ans;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(
    const Eigen::QuaternionBase<Derived>& p) {
  Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
  ans.template block<3, 1>(
      1, 0) = pp.vec(),
         ans.template block<3, 3>(1, 1) =
             pp.w() *
                 Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
             SkewSymmetric(pp.vec());
  return ans;
}

}  // namespace Eigen

#endif