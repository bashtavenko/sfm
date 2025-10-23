#include "image_processing_toolbox.h"
#include <filesystem>
#include "absl/strings/str_format.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace sfm {
absl::Status SaveFrames(absl::string_view video_path, int32_t k,
                        absl::string_view frame_output_directory) {
  if (k <= 0) {
    return absl::InvalidArgumentError(
        "Number of frames to extract must be positive.");
  }

  std::filesystem::path output_dir(frame_output_directory);
  if (!std::filesystem::exists(output_dir)) {
    if (!std::filesystem::create_directories(output_dir)) {
      return absl::InternalError("Failed to create output directory.");
    }
  }

  cv::VideoCapture cap(video_path.data());
  if (!cap.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open video file '%s'.", video_path));
  }

  int32_t frame_count = static_cast<int32_t>(cap.get(cv::CAP_PROP_FRAME_COUNT));
  if (frame_count <= 0) {
    return absl::InternalError("Failed to read video frame count.");
  }

  double step = static_cast<double>(frame_count) / k;
  for (int32_t i = 0; i < k; ++i) {
    int frame_index = static_cast<int>(i * step);
    cap.set(cv::CAP_PROP_POS_FRAMES, frame_index);

    cv::Mat frame;
    if (!cap.read(frame)) {
      return absl::InternalError(
          absl::StrFormat("Failed to read frame at index %i", frame_index));
    }

    std::ostringstream filename;
    filename << "frame_" << i << ".jpg";
    std::filesystem::path output_file = output_dir / filename.str();
    if (!cv::imwrite(output_file.string(), frame)) {
      return absl::InternalError(absl::StrFormat(
          "Failed to save frame to '%s'.", output_file.string()));
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<MotionSummary> ComputeMotionSummary(
    const cv::Mat& image_frame, const cv::Mat& previous_image_frame) {
  MotionSummary motion_summary;

  std::vector<cv::Point2f> corners_a;
  std::vector<cv::Point2f> corners_b;
  constexpr int32_t kMaxCorners = 500;
  cv::goodFeaturesToTrack(previous_image_frame, // Image to track
                          corners_a, // Vector of detected corners (output)
                          kMaxCorners, // Keep up to this many corners
                          0.01, // Quality level (percent of maximum)
                          5, // Min distance between corners
                          cv::noArray(), // Mask
                          3, // Block size
                          false, // true: Harris, false: Shi-Tomasi
                          0.04 // method specific parameter
      );

  constexpr int32_t win_size = 10; // compute local coherent motion
  cv::cornerSubPix(
      previous_image_frame, // Input image
      corners_a, // Vector of corners (input and output)
      cv::Size(win_size, win_size), // Half side length of search window
      cv::Size(-1, -1), // Half side length of dead zone (-1=none)
      cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                       20, // Maximum number of iterations
                       0.03 // Minimum change per iteration
          ));

  // Lucas Kanade algorithm
  std::vector<uchar> features_found;
  cv::calcOpticalFlowPyrLK(
      previous_image_frame,           // Previous image
      image_frame,           // Next image
      corners_a,       // Previous set of corners (from imgA)
      corners_b,       // Next set of corners (from imgB)
      features_found,  // Output vector, each is 1 for tracked
      cv::noArray(),   // Output vector, lists errors (optional)
      cv::Size(win_size * 2 + 1, win_size * 2 + 1),  // Search window size
      5,  // Maximum pyramid level to construct
      cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                       20,  // Maximum number of iterations
                       0.3  // Minimum change per iteration
                       ));

  return motion_summary;
}
} // namespace sfm