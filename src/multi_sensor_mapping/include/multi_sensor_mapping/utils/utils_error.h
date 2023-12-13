// #ifndef MSM_UTILS_ERROR_H
// #define MSM_UTILS_EROOR_H
#pragma once

#include <ctime>
#include <string>

namespace multi_sensor_mapping {

enum ErrCodeType {
  INFO_CODE,     // 0x00 ~ 0x3F
  WARNING_CODE,  // 0x40 ~ 0x7F
  ERROR_CODE     // 0x80 ~ 0xBF
};

enum ErrCode {
  // Info
  CODE_SUCCESS = 0X00,

  // Warning

  // Error

  CODE_START_BEFORE_INIT = 0x80,  // 在init()之前调用start()
  CODE_NO_INPUT_PARAMS = 0x81,    // 没有输入参数

};

struct MSMError {
  ErrCode code_;
  ErrCodeType code_type_;

  MSMError() : code_(ErrCode::CODE_SUCCESS) {}

  explicit MSMError(const ErrCode& _code) : code_(_code) {
    if (code_ < 0x40) {
      code_type_ = ErrCodeType::INFO_CODE;
    } else if (code_ < 0x80) {
      code_type_ = ErrCodeType::WARNING_CODE;
    } else {
      code_type_ = ErrCodeType::ERROR_CODE;
    }
  }

  std::string ToString() {
    switch (code_) {
      case CODE_SUCCESS:
        return "[MSM INFO] Success";

      default:
        break;
    }

    return "[EC INFO] Wrong error code";
  }
};

}  // namespace multi_sensor_mapping

// #endif