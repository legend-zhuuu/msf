#ifndef MSM_UTILS_YAML_H
#define MSM_UTILS_YAML_H

#include <yaml-cpp/yaml.h>

#include "multi_sensor_mapping/utils/utils_log.h"

/// adapte from  rslidar_sdk

namespace multi_sensor_mapping {

namespace utils {

/**
 * @brief 读取YAML数据，读取失败则退出
 *
 * @tparam T
 * @param yaml
 * @param key
 * @param out_val
 */
template <typename T>
inline void YamlReadAbort(const YAML::Node& yaml, const std::string& key,
                          T& out_val) {
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null) {
    AERROR_F(" Yaml read : Not set key [ %s ] , Aborting!!!", key.c_str());
    exit(-1);
  } else {
    out_val = yaml[key].as<T>();
  }
}

/**
 * @brief 读取YAML数据，若不存在，则使用默认值
 *
 * @tparam T
 * @param yaml
 * @param key
 * @param out_val
 * @param default_val
 * @return true
 * @return false
 */
template <typename T>
inline bool YamlRead(const YAML::Node& yaml, const std::string& key, T& out_val,
                     const T& default_val) {
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null) {
    out_val = default_val;
    return false;
  } else {
    out_val = yaml[key].as<T>();
    return true;
  }
}

/**
 * @brief YAML 子节点读取
 *
 * @param yaml
 * @param node
 * @return YAML::Node
 */
inline YAML::Node YamlSubNodeAbort(const YAML::Node& yaml,
                                   const std::string& node) {
  YAML::Node ret = yaml[node.c_str()];
  if (!ret) {
    AERROR_F(" Yaml Sub Node : Cannot find subnode [ %s ], Aborting!!!",
             node.c_str());
    exit(-1);
  }
  return ret;
}

/**
 * @brief 检测YAML子节点是否存在
 *
 * @param yaml
 * @param node
 * @return true
 * @return false
 */
inline bool CheckYamlSubNode(const YAML::Node& yaml, const std::string& node) {
  YAML::Node ret = yaml[node.c_str()];
  if (!ret) {
    return false;
  }
  return true;
}

}  // namespace utils

}  // namespace multi_sensor_mapping

#endif