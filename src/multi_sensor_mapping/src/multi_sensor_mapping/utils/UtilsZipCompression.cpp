#include "multi_sensor_mapping/utils/UtilsZipCompression.h"

namespace multi_sensor_mapping {

namespace utils {


ZipCompression* ZipCompression::instance_ = NULL;

ZipCompression* ZipCompression::GetInstance() {
    if (instance_ == NULL) {
        instance_ = new ZipCompression();
    }
    return instance_;
}

ZipCompression::ZipCompression() {}

void ZipCompression::FileZipCompression(const std::string _file_path, const std::string _zip_path) {

    zipper::Zipper zipper(_zip_path);

    zipper.add(_file_path);

    zipper.close();
}

void ZipCompression::MultipleFilesZipCompression(const std::vector<std::string>& _files_path, const std::string _zip_path) {
    zipper::Zipper zipper(_zip_path);

    for (auto iter = _files_path.begin(); iter != _files_path.end(); ++iter) {
        zipper.add(*iter);
    }

    zipper.close();
}

void ZipCompression::FileZipDecompress(const std::string _file_path, const std::string _zip_path) {
    
    zipper::Unzipper unzipper(_zip_path);

    unzipper.extractAll(_file_path);
    unzipper.extractAll(_file_path, true);

    unzipper.close();
}

}
}
