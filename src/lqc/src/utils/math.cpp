#include <lqc/utils/math.h>

namespace lqc {

bool legInvKin(std::size_t leg, const float pos[3], float angle[3]) {
  // leg: 0 = forward right; 1 = forward left;
  //      2 = rear right;    3 = rear left.
  constexpr float l1 = 0.083, l2 = 0.25, l3 = 0.25;
  constexpr float ls = l1 * l1 + l2 * l2 + l3 * l3;

  float px = pos[0], py = pos[1], pz = pos[2];
  float pyz2 = py * py + pz * pz, pxyz2 = px * px + pyz2;
  if (pyz2 < l1 * l1) return false;
  float l1_dy = leg % 2 ? l1 : -l1, py_offset = py + l1_dy;
  if (std::abs(py_offset) > 0.001) {
    angle[0] = 2 * std::atan(
        (pz + std::sqrt(pyz2 - l1 * l1)) / py_offset
    );
  } else {
    angle[0] = 2 * std::atan(
        l1_dy / pz // limit
    );
  }

  float cos3 = (pxyz2 - ls) / (2 * l2 * l3);
  if (std::abs(cos3) > 1) return false;
  angle[2] = -std::acos(cos3);
  float pr = std::sqrt(pxyz2 - l1 * l1);
  float sin1 = px / pr, sin2 = l3 * std::sin(angle[2]) / pr;
  if (std::abs(sin1) > 1 or std::abs(sin2) > 1) return false;
  angle[1] = -std::asin(sin1) - std::asin(sin2);
  return true;
}

bool invKin(const float pos[12], float angle[12]) {
  for (int i = 0; i < 4; ++i) {
    if (!legInvKin(i, pos + i * 3, angle + i * 3)) {
      return false;
    }
  }
  return true;
}

Quaternion Quaternion::fromRpy(cVec3 rpy) {
  return Quaternion(std::cos(rpy[2] / 2), 0., 0., std::sin(rpy[2] / 2)) *
      Quaternion(std::cos(rpy[1] / 2), 0., std::sin(rpy[1] / 2), 0.) *
      Quaternion(std::cos(rpy[0] / 2), std::sin(rpy[0] / 2), 0., 0.);
}

Quaternion &Quaternion::operator=(const Eigen::Ref<const Vec4> &q) {
  eigen_ref() << q;
  return *this;
}

Quaternion Quaternion::operator*(const Quaternion &other) const {
  return {w * other.w - x * other.x - y * other.y - z * other.z,
          w * other.x + x * other.w + y * other.z - z * other.y,
          w * other.y - x * other.z + y * other.w + z * other.x,
          w * other.z + x * other.y - y * other.x + z * other.w};
}

Mat3 Quaternion::mat() const {
  Mat3 R;
  R(0, 0) = 1 - 2 * (y * y + z * z);
  R(0, 1) = 2 * (x * y - w * z);
  R(0, 2) = 2 * (x * z + w * y);
  R(1, 0) = 2 * (x * y + w * z);
  R(1, 1) = 1 - 2 * (x * x + z * z);
  R(1, 2) = 2 * (y * z - w * x);
  R(2, 0) = 2 * (x * z - w * y);
  R(2, 1) = 2 * (y * z + w * x);
  R(2, 2) = 1 - 2 * (x * x + y * y);
  return R;
}

Vec3 Quaternion::rpy() const {
  return {
      std::atan2(2 * (w * x + y * z),
                 1 - 2 * (x * x + y * y)),
      -M_PIf32 / 2 + 2 * std::atan2(
          std::sqrt(1 + 2 * (w * y - x * z)),
          std::sqrt(1 - 2 * (w * y - x * z))),
      std::atan2(2 * (w * z + x * y),
                 1 - 2 * (y * y + z * z)),
  };
}

Vec3 Quaternion::rotate(cVec3 v) const {
  Vec3 u{x, y, z};
  return 2.0f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.0f * w * u.cross(v);
}

}  // namespace lqc
