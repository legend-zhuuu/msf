#ifndef LQC_UTILS_EV_H_
#define LQC_UTILS_EV_H_

#include <string>

namespace lqc {
namespace ev {

enum Status { Success, NotFound, Invalid };
Status parse(const char *name, std::string &result);
Status parse(const char *name, long &result);

}  // namespace ev
}  // namespace lqc

#endif  // LQC_UTILS_EV_H_
