#include "sfm.h"
#include <google/protobuf/text_format.h>
#include <filesystem>
#include "glog/logging.h"
#include <fstream>

namespace sfm {

absl::StatusOr<std::vector<std::string>> GetFilesFromDirectory(
    absl::string_view dir) {
  std::vector<std::string> files;

  if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Directory '%s' does not exist or is not a directory.", dir));
  }
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
      files.push_back(entry.path().string());
    }
  }
  return files;
}

absl::StatusOr<proto::CameraMatrix> LoadCameraMatrixFromTextProtoFile(
    absl::string_view file_path) {
  std::ifstream file(file_path.data());
  std::string text_proto((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());
  proto::CalibrationResult message;
  if (!google::protobuf::TextFormat::ParseFromString(text_proto.data(),
                                                     &message)) {
    return absl::InternalError("Failed to parse proto message");
  }
  return message.camera_matrix();
}

}  // namespace sfm
