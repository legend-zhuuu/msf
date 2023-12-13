#include <cstdlib>
#include <lqc/utils/ev.h>

namespace lqc {
namespace ev {

Status parse(const char *name, std::string &result) {
  const char *var = std::getenv(name);
  if (!var) return NotFound;
  result = var;
  return Success;
}

Status parse(const char *name, long &result) {
  const char *var = std::getenv(name);
  if (!var) return NotFound;
  char *end{nullptr};
  long _result = std::strtol(var, &end, 10);
  if (*end != '\0') return Invalid;
  result = _result;
  return Success;
}

}  // namespace ev
}  // namespace lqc