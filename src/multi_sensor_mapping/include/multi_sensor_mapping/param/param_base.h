#ifndef MSM_PARAM_BASE_H
#define MSM_PARAM_BASE_H

#include <iostream>

#include "multi_sensor_mapping/utils/utils_common.h"

namespace multi_sensor_mapping {

/**
 * @brief The ParamBase class 参数类的基类，定义新的参数类时，
 * 需要继承该基类并重写以下三个虚函数
 */
class ParamBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ParamBase> Ptr;

 public:
  virtual void Load(std::string _path_of_yaml) = 0;
  virtual void Print() = 0;

  /**
   * @brief Type 返回该参数的类型
   * @return
   */
  virtual ParamType Type() const = 0;

  /**
   * @brief Name 返回该参数的名称，用于GUI的显示与保存的文件名等
   * @return
   */
  virtual std::string Name() const = 0;

 protected:
  template <typename T>
  void PrintLine(const std::string& _id, const T& _input) const {
    if (std::is_same<typename std::decay<T>::type, bool>::value) {
      std::cout << _id << ": " << std::boolalpha << _input << std::endl;
    } else {
      std::cout << _id << ": " << _input << std::endl;
    }
  }

 protected:
  /// @brief 参数的名称，用于GUI的显示与保存的文件名等
  std::string name_;
};

}  // namespace multi_sensor_mapping

#endif
