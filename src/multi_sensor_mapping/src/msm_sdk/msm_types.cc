#include "msm_sdk/msm_types.h"

#include <string>

namespace msm_sdk {

Version::Version(int _marjor, int _minor, int _patch) {
  major_ = _marjor;
  minor_ = _minor;
  patch_ = _patch;
}

std::string Version::toString() {
  std::string version_str = std::string("v.") + std::to_string(major_) +
                            std::string(".") + std::to_string(minor_) +
                            std::string(".") + std::to_string(patch_);
  return version_str;
}

}  // namespace msm_sdk