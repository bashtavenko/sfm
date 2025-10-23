#ifndef IMAGE_PROCESSING_TOOLBOX_H
#define IMAGE_PROCESSING_TOOLBOX_H
#include <filesystem>
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/status/statusor.h"
#include "opencv2/opencv.hpp"

namespace sfm {
// Saves k frames from the given video into output directory
absl::Status SaveFrames(absl::string_view video_path, int32_t k,
                        absl::string_view frame_output_directory);

// Finds quality frames for structure from motion.
// Frames should have different camera poses and have overlap.
// Depending on criteria the number of output frames varies and can zero.
absl::Status SaveQualityFrames(absl::string_view video_path,
                        absl::string_view frame_output_directory);

struct MotionSummary {
  enum class MotionType {
    // Very low motion magnitude (< 1.0 pixels)
    kMinimal,
    // Low standard deviation in flow components, indicating uniform motion
    // (translation, rotation, or scaling)
    kRigid,
    // High variation in flow field, suggesting non-rigid or complex motion
    kDeformative
  };
  float mean_keypoint_dx_pixels;
  float mean_keypoint_dy_pixels;
  float previous_keypoint_size;
  // Mean motion by dx;
  float mean_dx;
  // Mean motion by dy;
  float mean_dy;
  // Mean magnitude (sqrt(dx^2 + dy^2)
  float mean_magnitude;
  // Mean direction in degrees.
  float mean_direction;
  float std_dx;
  float std_dy;
  MotionType motion_type;
};

absl::StatusOr<MotionSummary>ComputeMotionSummary(cv::Mat& image_frame, cv::Mat& previous_image_frame);


} // namespace sfm

#endif  // IMAGE_PROCESSING_TOOLBOX_H
