#ifndef LQC_UTILS_YAML_H_
#define LQC_UTILS_YAML_H_

#include <cxxabi.h>
#include <yaml-cpp/yaml.h>
#include <lqc/utils/definitions.h>

#define NODE_REF(node) "\"\"\"\n" << node << "\n\"\"\""

namespace lqc {
/**
 * @brief This namespace contains utility functions for working with YAML.
 */
namespace yaml {

using Node = YAML::Node;

// assert(node)
inline void assertValid(const Node &node) {
  LQC_ASSERT(node, "Invalid node.")
}

// assert(node[key])
template<typename Key>
void assertValid(const Node &node, const Key &key) {
  assertValid(node);
  LQC_ASSERT(node[key], "Key `" << key << "` does not exist in node " << NODE_REF(node) << ".")
}

/**
 * @brief Checks if a YAML node can be converted to a given type.
 *
 * @tparam T The type to check for.
 * @param node The YAML node to check.
 * @return true if the node can be converted to the given type, false otherwise.
 */
template<typename T>
bool isType(const Node &node) {
  try {
    node.as<T>();
    return true;
  } catch (...) {
    return false;
  }
}

inline bool isBool(const Node &node) { return isType<bool>(node); }
inline bool isFloat(const Node &node) { return isType<double>(node); }

inline bool isNTuple(const Node &node, std::size_t size) {
  return node.IsSequence() && node.size() == size;
}

inline void assertNTuple(const Node &node, std::size_t size) {
  LQC_ASSERT(isNTuple(node, size),
             "Node " NODE_REF(node) " requires to be a " << size << "-tuple.")
}

template<typename Key>
inline void assertNTuple(const Node &node, const Key &key, std::size_t size) {
  LQC_ASSERT(isNTuple(node[key], size),
             "`" << key << "` requires to be a " << size << "-tuple.")
}

namespace impl {
template<typename V, typename Key>
V readAs(const Node &node, const Key &key) {
  assertValid(node, key);
  return node[key].template as<V>();
}

// value = node.as<V>();
template<typename V>
void setTo(const Node &node, V &value) {
  try {
    value = node.template as<V>();
  } catch (const std::runtime_error &e) {  /// gives a more detailed information
    int status;
    char *type_name = abi::__cxa_demangle(typeid(V).name(), nullptr, nullptr, &status);
    LQC_ERROR("Bad conversion of Node " << NODE_REF(node) << "to type `" << type_name << "`.")
  }
}

template<typename T>
void setTo(const Node &node, std::vector<T> &value) {
  value.clear();
  if (node.IsScalar()) {
    value.resize(1);
    return setTo(node, value.back());
  }
  value.resize(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    setTo(node[i], value[i]);
  }
}

template<typename T, int N>
void setTo(const Node &node, T value[N]) {
  static_assert(N != 0, "setTo: Invalid array size 0.");
  if (node.IsScalar()) {
    setTo(node, value[0]);
    std::fill(value + 1, value + N, value[0]);
    return;
  }
  assertNTuple(node, N);
  for (std::size_t i = 0; i < N; ++i) {
    setTo(node[i], value[i]);
  }
}

template<typename T, int N>
void setTo(const Node &node, Eigen::Array<T, N, 1> &value) {
  if (node.IsScalar()) {
    T scalar;
    setTo(node, scalar);
    value.setConstant(scalar);
    return;
  }
  std::size_t node_size = node.size();
  LQC_ASSERT(value.size() == node_size, "setTo: Invalid node size.")
  for (size_t i = 0; i < node_size; i++) {
    setTo(node[i], value[i]);
  }
}

template<typename V>
void checkAndSet(bool assert_exists, const Node &node, V &value) {
  if (node) return setTo(node, value);
  if (assert_exists) assertValid(node);
}

template<typename V, typename Key>
void checkAndSet(bool assert_exists, const Node &node, const Key &key, V &value) {
  if (node && node[key]) return setTo(node[key], value);
  if (assert_exists) assertValid(node, key);
}

template<typename V, std::size_t N, typename Key>
void checkAndSet(bool assert_exists, const Node &node, const Key &key, V value[N]) {
  if (node && node[key]) return setTo<V, N>(node[key], value);
  if (assert_exists) assertValid(node, key);
}

} // namespace impl

// assert(node); value = node.as<V>();
template<typename V>
inline void setTo(const Node &node, V &value) {
  impl::checkAndSet(true, node, value);
}
// assert(node && node[key]); value = node[key].as<V>();
template<typename V, typename Key>
inline void setTo(const Node &node, const Key &key, V &value) {
  impl::checkAndSet(true, node, key, value);
}

template<typename V, std::size_t N, typename Key>
inline void setTo(const Node &node, const Key &key, V value[N]) {
  impl::checkAndSet<V, N, Key>(true, node, key, value);
}

// if (node) value = node.as<V>();
template<typename V>
inline void setIfValid(const Node &node, V &value) {
  impl::checkAndSet(false, node, value);
}
// if (node && node[key]) value = node[key].as<V>();
template<typename V, typename Key>
inline void setIfValid(const Node &node, const Key &key, V &value) {
  impl::checkAndSet(false, node, key, value);
}

template<typename V, std::size_t N, typename Key>
inline void setIfValid(const Node &node, const Key &key, V value[N]) {
  impl::checkAndSet<V, N, Key>(false, node, key, value);
}

/**
 * @brief Sets a complex parameter based on the given YAML node.
 *
 * If the node is a boolean, sets the 'on' parameter to the value of the boolean.
 * Otherwise, sets the 'on' parameter to true and sets the 'param' parameter to the value of the node.
 *
 * @tparam V The type of the 'param' parameter.
 * @param node The YAML node to set the parameter from.
 * @param on A boolean parameter indicating whether the parameter is on or off.
 * @param param The parameter to set.
 */
template<typename V>
void setComplex(const Node &node, bool &on, V &param) {
  if (!node) return;
  if (isBool(node)) {
    impl::setTo(node, on);
  } else {
    on = true;
    impl::setTo(node, param);
  }
}
template<typename V, typename Key>
void setComplex(const Node &node, const Key &key, bool &on, V &param) {
  if (!node) return;
  setComplex(node[key], on, param);
}

template<typename Key>
Node read(const Node &node, const Key &key) {
  assertValid(node, key);
  return node[key];
}

template<typename V>
V readAs(const Node &node) {
  V value;
  setTo(node, value);
  return std::move(value);
}

template<typename V, typename Key>
V readAs(const Node &node, const Key &key) {
  V value;
  setTo(node, key, value);
  return std::move(value);
}

template<typename V>
V readDefault(const Node &node, const V &default_value) {
  return node ? readAs<V>(node) : default_value;
}

template<typename V, typename Key>
V readDefault(const Node &node, const Key &key, const V &default_value) {
  return node && node[key] ? readAs<V>(node[key]) : default_value;
}

inline bool isOn(const Node &node) {
  return readDefault(node, true);
}

template<typename Key>
inline bool isOn(const Node &node, const Key &key) {
  return readDefault(node, key, true);
}

inline bool isOff(const Node &node) {
  return readDefault(node, false);
}

template<typename Key>
inline bool isOff(const Node &node, const Key &key) {
  return readDefault(node, key, false);
}

template<typename Key>
inline bool isValid(const Node &node, const Key &key) {
  return node && node[key];
}

template<typename Key, typename... SeqKeys>
void isValid(const Node &node, const Key &key, SeqKeys... seq_keys) {
  return node && isValid(node[key], seq_keys...);
}

} // namespace yaml
} // namespace lqc

#undef NODE_REF

#endif // LQC_UTILS_YAML_H_
