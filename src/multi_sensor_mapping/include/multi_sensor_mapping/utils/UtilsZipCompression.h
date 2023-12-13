#ifndef UTILSZIPCOMPRESSION_H
#define UTILSZIPCOMPRESSION_H

#include <Zipper/Unzipper.hpp>
#include <Zipper/Zipper.hpp>

#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace multi_sensor_mapping {

namespace utils {

/**
 * @brief The ZIP Compression class 文件ZIP压缩（单例）
 */

class ZipCompression {
  public:
    static ZipCompression *GetInstance();

  /**
   * @brief FileZipCompression ZIP文件压缩
   * @param _file_path
   * @param _zip_path
   */
  void FileZipCompression(const std::string _file_path, const std::string _zip_path);

  /**
   * @brief MultipleFilesZipCompression ZIP多文件压缩
   * @param _files_path
   * @param _zip_path
   */
  void MultipleFilesZipCompression(const std::vector<std::string>& _files_path, const std::string _zip_path);
  /**
   * @brief FileZipDecompress Zip文件解压缩
   * @param _file_path
   * @param _zip_path
   */
  void FileZipDecompress(const std::string _file_path, const std::string _zip_path);

private:
/**
 * @brief ZipCompression 构造函数
 */
ZipCompression();

/// @brief 实例
static ZipCompression *instance_;

};
}
}

#endif // UTILSZIPCOMPRESSION_H
