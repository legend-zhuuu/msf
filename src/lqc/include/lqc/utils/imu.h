#ifndef LQC_UTILS_IMU_H_
#define LQC_UTILS_IMU_H_

#include <lqc/utils/definitions.h>
#include <lqc/utils/math.h>
#include <lqc/utils/static_queue.hpp>

namespace lqc {

class SimpleAccelFilter {
 public:
  explicit SimpleAccelFilter(int buffer_size = 10);
  void reset();
  void update(const sdk::LowState &);
  const Eigen::Array3f &getAccel() const;
  Eigen::Array3f getFilteredAccel() const;

 private:
  Quaternion orn_;
  Eigen::Array3f gravity_{0., 0., -9.81};
  StaticQueue<Eigen::Array3f> accel_buf_;
  Eigen::Array3f accel_;
};

}  // namespace lqc

#endif  // LQC_UTILS_IMU_H_
