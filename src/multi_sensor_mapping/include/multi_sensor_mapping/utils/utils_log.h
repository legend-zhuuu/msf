#ifndef MSM_UTILS_LOG_H
#define MSM_UTILS_LOG_H

#include <cstdarg>
#include <string>

#include "multi_sensor_mapping/utils/loguru.h"

#define ADEBUG LOG_S(1) << "[DEBUG] "
#define AINFO LOG_S(INFO)
#define AWARN LOG_S(WARNING)
#define AERROR LOG_S(ERROR)
#define AFATAL LOG_S(FATAL)

#define ADEBUG_F(...) LOG_F(1, __VA_ARGS__)
#define AINFO_F(...) LOG_F(INFO, __VA_ARGS__)
#define AWARN_F(...) LOG_F(WARNING, __VA_ARGS__)
#define AERROR_F(...) LOG_F(ERROR, __VA_ARGS__)
#define AFATAL_F(...) LOG_F(FATAL, __VA_ARGS__)

// #define ADEBUG VLOG(4) << "[DEBUG] "
// #define AINFO ALOG(INFO)
// #define AWARN ALOG(WARN)
// #define AERROR ALOG(ERROR)
// #define AFATAL ALOG(FATAL)

// #ifndef ALOG_STREAM
// #define ALOG_STREAM(log_severity) ALOG_STREAM_##log_severity
// #endif

// #ifndef ALOG
// #define ALOG(log_severity) ALOG_STREAM(log_severity)
// #endif

// #define ALOG_STREAM_INFO
//   google::LogMessage(__FILE__, __LINE__, google::INFO).stream()

// #define ALOG_STREAM_WARN
//   google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()

// #define ALOG_STREAM_ERROR
//   google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()

// #define ALOG_STREAM_FATAL
//   google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()

// #define AINFO_IF(cond) ALOG_IF(INFO, cond)
// #define AWARN_IF(cond) ALOG_IF(WARN, cond)
// #define AERROR_IF(cond) ALOG_IF(ERROR, cond)
// #define AFATAL_IF(cond) ALOG_IF(FATAL, cond)
// #define ALOG_IF(severity, cond) !(cond) ? (void)0 :
// google::LogMessageVoidify() & ALOG(severity)

// #define ACHECK(cond) CHECK(cond)

#define ACHECK(cond) CHECK_S(cond)
#define ACHECK_F(cond, ...) CHECK_F(cond, ##__VA_ARGS__)

#define ACHECK_EQ(expr1, expr2) CHECK_EQ_S(expr1, expr2)
#define ACHECK_EQ_F(a, b, ...) CHECK_EQ_F(a, b, ##__VA_ARGS__)

#define ACHECK_GE(expr1, expr2) CHECK_GE_S(expr1, expr2)
#define ACHECK_GE_F(a, b, ...) CHECK_GE_F(a, b, ##__VA_ARGS__)

#define ACHECK_NOTNULL(x) CHECK_NOTNULL_S(x)
#define ACHECK_NOTNULL_F(x, ...) CHECK_NOTNULL_F(x, ##__VA_ARGS__)

// #define AINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
// #define AWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
// #define AERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

// #if !defined(RETURN_IF_NULL)
// #define RETURN_IF_NULL(ptr)
//   if (ptr == nullptr) {
//     AWARN << #ptr << " is nullptr.";
//     return;
//   }
// #endif

// #if !defined(RETURN_VAL_IF_NULL)
// #define RETURN_VAL_IF_NULL(ptr, val)
//   if (ptr == nullptr) {
//     AWARN << #ptr << " is nullptr.";
//     return val;
//   }
// #endif

// #if !defined(RETURN_IF)
// #define RETURN_IF(condition)
//   if (condition) {
//     AWARN << #condition << " is met.";
//     return;
//   }
// #endif

// #if !defined(RETURN_VAL_IF)
// #define RETURN_VAL_IF(condition, val)
//   if (condition) {
//     AWARN << #condition << " is met.";
//     return val;
//   }
// #endif

// #if !defined(_RETURN_VAL_IF_NULL2__)
// #define _RETURN_VAL_IF_NULL2__
// #define RETURN_VAL_IF_NULL2(ptr, val)
//   if (ptr == nullptr) {
//     return (val);
//   }
// #endif

// #if !defined(_RETURN_VAL_IF2__)
// #define _RETURN_VAL_IF2__
// #define RETURN_VAL_IF2(condition, val)
//   if (condition) {
//     return (val);
//   }
// #endif

// #if !defined(_RETURN_IF2__)
// #define _RETURN_IF2__
// #define RETURN_IF2(condition)
//   if (condition) {
//     return;
//   }
// #endif

#endif