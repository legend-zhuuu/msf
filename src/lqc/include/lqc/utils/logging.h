#ifndef LQC_UTILS_LOGGING_H_
#define LQC_UTILS_LOGGING_H_

#include <iostream>
#include <mutex>
#include <sstream>

namespace lqc {

template<typename Iter>
struct Segment {
  Segment(Iter begin, Iter end) : begin(begin), end(end) {}
  Iter begin, end;
};

template<typename Iter>
Segment<Iter> make_segment(Iter begin, Iter end) {
  return {begin, end};
}

template<typename Iter>
std::ostream &operator<<(std::ostream &os, const Segment<Iter> &container) {
  if (container.begin == container.end) return os << "[]";
  os << "[" << *container.begin;
  for (auto it = container.begin + 1; it != container.end; ++it) {
    os << ", " << *it;
  }
  return os << "]";
}

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &vec) {
  return os << make_segment(vec.begin(), vec.end());
}

namespace lg {

enum Level {
  WARN = 0,
  INFO = 1,
  DBUG = 2,
};

bool inMode(Level level);
void setLevel(Level level);
void setLevel(long level);
void enableTimeStampPrefix(bool enabled);

void _log(const std::string &info);

template<typename...Args>
std::string toStr(Args &&...args) {
  std::stringstream ss;
  int _[]{(ss << std::forward<Args>(args), 0)...};
  return ss.str();
}

template<typename...Args>
void log(Args &&...args) {
  _log(toStr(args...));
}

template<typename...Args>
void dbug(Args &&...args) {
  if (!inMode(DBUG)) return;
  _log(toStr(args...));
}

template<typename...Args>
void info(Args &&...args) {
  if (!inMode(INFO)) return;
  _log(ESC_GRN + toStr(args...) + ESC_CLR);
}

template<typename...Args>
void warn(Args &&...args) {
  if (!inMode(WARN)) return;
  _log(ESC_YLW + toStr(args...) + ESC_CLR);
}

template<typename...Args>
void crit(Args &&...args) {
  _log(ESC_RED + toStr(args...) + ESC_CLR);
}

}  // namespace lg

using lg::log;

}  // namespace lqc

#endif  // LQC_UTILS_LOGGING_H_
