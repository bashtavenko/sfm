#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "absl/status/statusor.h"
#include "calibration_data.pb.h"

namespace sfm {

// Calibrate intrinsict from directory of chessboard images.
absl::StatusOr<proto::CalibrationResult> Calibrate(absl::string_view images_directory);

}  // namespace sfm

#endif  // CALIBRATOR_H
