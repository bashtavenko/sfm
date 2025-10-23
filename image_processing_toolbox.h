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

  // Ratio of tracked keypoints sizes vs previous keypoints sizes
  // If image is the same ratio will be 1.
  float tracked_points_ratio = 0;
  // Mean motion by dx;
  float mean_dx = 0;
  // Mean motion by dy;
  float mean_dy = 0;
  // Mean magnitude (sqrt(dx^2 + dy^2)
  float mean_magnitude = 0;
  // Mean direction in degrees.
  float mean_direction = 0;
  float std_dx = 0;
  float std_dy = 0;
  MotionType motion_type = MotionType::kMinimal;
};

MotionSummary ComputeMotionSummary(const cv::Mat& image_frame,
                                   const cv::Mat& previous_image_frame);
} // namespace sfm

#endif  // IMAGE_PROCESSING_TOOLBOX_H
