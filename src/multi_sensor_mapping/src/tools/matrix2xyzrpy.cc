#include "multi_sensor_mapping/utils/utils_geometry.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  Eigen::Matrix4d mat;
  mat << -9.99957263e-01, -6.27219537e-03, 6.79167639e-03, 3.90605927e-01,
      -6.64514862e-03, -2.30940506e-02, -9.99711215e-01, -3.44702080e-02,
      6.42723171e-03, -9.99713600e-01, 2.30513848e-02, -1.40744896e-05, 0., 0.,
      0., 1.;

  double x, y, z, roll, pitch, yaw;
  utils::Transd2XYZYPR(mat, x, y, z, yaw, pitch, roll);

  std::cout << "x : " << x << std::endl;
  std::cout << "y : " << y << std::endl;
  std::cout << "z : " << z << std::endl;
  std::cout << "roll : " << roll << std::endl;
  std::cout << "pitch : " << pitch << std::endl;
  std::cout << "yaw : " << yaw << std::endl;

  Eigen::Matrix4d matrix = utils::XYZYPR2Transd(x, y, z, yaw, pitch, roll);
  std::cout << matrix << std::endl;

  return 0;
}