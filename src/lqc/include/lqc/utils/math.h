#ifndef LQC_UTILS_IK_H_
#define LQC_UTILS_IK_H_

#include <cmath>
#include <lqc/utils/definitions.h>

namespace lqc {

bool legInvKin(std::size_t leg, const float pos[3], float angle[3]);
bool invKin(const float pos[12], float angle[12]);

struct Quaternion {
  Quaternion() : Quaternion(1.f, 0.f, 0.f, 0.f) {}
  Quaternion(float w, float x, float y, float z) : w{w}, x{x}, y{y}, z{z} {}
  auto eigen_ref() { return Eigen::Map<Vec4>{ptr}; }
  Quaternion(cVec4 q) { eigen_ref() << q; }
  Quaternion(const Quaternion &q) : Quaternion(q.w, q.x, q.y, q.z) {}
  static Quaternion fromRpy(cVec3 rpy);

  void set(float w, float x, float y, float z) { eigen_ref() << w, x, y, z; }
  void setIdentity() { eigen_ref() << 1., 0., 0., 0.; }
  float norm() const { return std::sqrt(squaredNorm()); }
  float squaredNorm() const { return w * w + x * x + y * y + z * z; }
  Quaternion &normalize() { return operator/=(norm()); }

  Quaternion &operator=(cVec4 q);
  Quaternion operator*(const Quaternion &other) const;
  Quaternion operator*(float coef) const { return {w * coef, x * coef, y * coef, z * coef}; }
  Quaternion operator/(float coef) const { return operator*(1 / coef); }
  Quaternion &operator*=(float coef) { return operator=(operator*(coef)); }
  Quaternion &operator/=(float coef) { return operator*=(1 / coef); }
  friend std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
    return os << "Quaternion(w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z << ")";
  }

  Vec4 eigen() const { return {w, x, y, z}; }
  Quaternion inv() const { return {w, -x, -y, -z}; }
  Mat3 mat() const;
  Vec3 rpy() const;
  Vec3 rotate(cVec3 v) const;

  union {
    struct { float w, x, y, z; };
    float ptr[4];
  };
};

template<typename T>
inline T clip(T val, T min, T max) {
  return std::min(std::max(val, min), max);
}

}  // namespace lqc

#endif  // LQC_UTILS_IK_H_
