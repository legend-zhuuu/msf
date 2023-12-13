#include "msm_sdk/msm_sdk.h"

#include "multi_sensor_mapping/utils/utils_log.h"
#include "version.h"

namespace msm_sdk {

Version GetVersion() {
  return Version(int(PROJECT_VERSION_MAJOR), int(PROJECT_VERSION_MINOR),
                 int(PROJECT_VERSION_PATCH));
}

void SetLogPath(std::string _log_path) {
  loguru::add_file(_log_path.c_str(), loguru::Append, loguru::Verbosity_MAX);
}
}  // namespace msm_sdk