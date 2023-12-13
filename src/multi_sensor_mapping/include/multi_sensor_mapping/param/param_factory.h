#ifndef MSM_PARAM_FACTORY_H
#define MSM_PARAM_FACTORY_H

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief The ParamFactory class
 * 各参数的工厂类
 */
class ParamFactory {
 public:
  /**
   * @brief Construct a new Param Factory object
   *
   */
  ParamFactory() {}

  /**
   * @brief Get the Instance object
   *
   * @return std::shared_ptr<ParamFactory>
   */
  static std::shared_ptr<ParamFactory> GetInstance();

  /**
   * @brief 新建参数
   *
   * @param _param_type
   * @return ParamBase::Ptr
   */
  ParamBase::Ptr NewParam(ParamType _param_type);

  /**
   * @brief 新建参数
   *
   * @param _param_type
   * @param _name
   * @return ParamBase::Ptr
   */
  ParamBase::Ptr NewParam(ParamType _param_type, std::string _name);

 private:
  static std::shared_ptr<ParamFactory> param_generation_;
};

}  // namespace multi_sensor_mapping

#endif
