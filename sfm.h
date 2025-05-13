// Common utilities
#ifndef SFM_H
#define SFM_H
#include "calibration_data.pb.h"
#include "absl/strings/string_view.h"
#include "absl/status/statusor.h"

namespace sfm {
absl::StatusOr<std::vector<std::string>> GetFilesFromDirectory(
    absl::string_view dir);
absl::StatusOr<proto::CameraMatrix> LoadCameraMatrixFromTextProtoFile(
    absl::string_view file_path);

}  // namespace sfm

#endif  // SFM_H
