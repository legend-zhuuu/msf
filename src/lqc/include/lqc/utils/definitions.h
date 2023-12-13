#ifndef LQC_UTILS_DEFINITIONS_H_
#define LQC_UTILS_DEFINITIONS_H_

#include <iostream>
#include <sstream>
#include <lqc/utils/types.h>

#define ESC_CLR "\033[0m"
#define ESC_RED "\033[1;31m"
#define ESC_GRN "\033[1;32m"
#define ESC_YLW "\033[1;33m"

#ifndef __FILE_NAME__
#define __FILE_NAME__ __FILE__
#endif

#define I_TO_STR(...) #__VA_ARGS__
#define TO_STR(text) I_TO_STR(text)

#define LQC_ERROR(msg) {                                                            \
  std::stringstream __ss;                                                           \
  __ss << ESC_RED "[" __FILE_NAME__ ":" TO_STR(__LINE__) "] " << msg << ESC_CLR;    \
  throw std::runtime_error(__ss.str());                                             \
}

#define LQC_ASSERT_NOT(cond, msg) if (cond) LQC_ERROR(msg)
#define LQC_ASSERT(cond, msg) LQC_ASSERT_NOT(!(cond), msg)
#define LQC_EPS 1e-8

#define LQC_ALIENGO 0
#define LQC_GO1 1
#define LQC_B1 2
#ifndef LQC_ROBOT
#define LQC_ROBOT LQC_ALIENGO
#endif

#if LQC_ROBOT == LQC_ALIENGO
#define LQC_ROBOT_NAME "aliengo"
#elif LQC_ROBOT == LQC_GO1
#define LQC_ROBOT_NAME "go1"
#elif LQC_ROBOT == LQC_B1
#define LQC_ROBOT_NAME "b1"
#endif  // LQC_ROBOT

#endif  // LQC_UTILS_DEFINITIONS_H_
