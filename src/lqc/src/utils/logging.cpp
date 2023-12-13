#include <atomic>
#include <lqc/utils/utils.h>
#include "lqc/utils/logging.h"

namespace lqc {
namespace lg {

std::atomic<Level> kLogLevel{INFO};
std::atomic<bool> kTimeStampPrefix{true};
std::mutex kLogMtx;

bool inMode(Level level) {
  return kLogLevel >= level;
}

void setLevel(Level level) {
  kLogLevel = level;
}

void setLevel(long level) {
  kLogLevel = Level(clip(level, 0l, 2l));
}

std::string getTimeStampStr() {
  auto time_stamp_ms = std::chrono::high_resolution_clock::now()
      .time_since_epoch().count() / 1'000'000;
  auto time_stamp_str = std::to_string(time_stamp_ms);
  time_stamp_str.insert(time_stamp_str.size() - 3, ".");
  return time_stamp_str;
}

void enableTimeStampPrefix(bool enabled) {
  kTimeStampPrefix = enabled;
}

void _log(const std::string &info) {
  LockGuard lock(kLogMtx);
  if (kTimeStampPrefix) {
    std::cout << "[" << getTimeStampStr() << "] ";
  }
  std::cout << info << std::endl;
}

}  // namespace lg
}  // namespace lqc
