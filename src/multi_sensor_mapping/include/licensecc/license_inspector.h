/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef LICENSE_INSPECTOR_H
#define LICENSE_INSPECTOR_H

#include "licensecc/licensecc.h"
#include <iostream>
#include <unordered_map>

/**
 * @brief The LicenseInspector class 软件授权检查
 */
class LicenseInspector {
public:
  /**
   * @brief LicenseInspector
   */
  LicenseInspector();

  /**
   * @brief PrintPcIdentifier 打印PC标识符
   */
  void PrintPcIdentifier();

  /**
   * @brief VerifyLicense 验证许可证
   * @param _lic_path
   * @return
   */
  bool VerifyLicense(std::string _lic_path);

private:
  /// @brief string_by_event_type_ 软件提示
  std::unordered_map<LCC_EVENT_TYPE, std::string> string_by_event_type_;

  /// @brief 机器指纹长度
  size_t pc_id_size_;
};

#endif
