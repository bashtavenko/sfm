#include "image_processing_toolbox.h"
#include <filesystem>
#include "absl/strings/str_format.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "absl/log/check.h"
#include <numeric>

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

MotionSummary CalculateFromLK(const std::vector<cv::Point2f>& prev_pts,
                              const std::vector<cv::Point2f>& next_pts,
                              const std::vector<uchar>& features_found) {
  std::vector<float> dx_values;
  std::vector<float> dy_values;
  std::vector<float> magnitudes;
  MotionSummary motion;

  // Collect motion vectors for successfully tracked points
  for (size_t i = 0;
       i < prev_pts.size() && i < next_pts.size() && i < features_found.size();
       ++i) {
    if (features_found[i]) {
      // Only use successfully tracked points
      float dx = next_pts[i].x - prev_pts[i].x;
      float dy = next_pts[i].y - prev_pts[i].y;
      float magnitude = sqrt(dx * dx + dy * dy);

      dx_values.push_back(dx);
      dy_values.push_back(dy);
      magnitudes.push_back(magnitude);
    }
  }

  if (dx_values.empty()) {
    return motion;
  }

  // Calculate mean motion
  motion.mean_dx = std::accumulate(dx_values.begin(), dx_values.end(), 0.0f) /
                   dx_values.size();
  motion.mean_dy = std::accumulate(dy_values.begin(), dy_values.end(), 0.0f) /
                   dy_values.size();
  motion.tracked_points_ratio = static_cast<float>(dx_values.size()) / prev_pts.size();
  motion.mean_magnitude = sqrt(
      motion.mean_dx * motion.mean_dx + motion.mean_dy * motion.mean_dy);
  motion.mean_direction =
      std::atan2(motion.mean_dy, motion.mean_dx) * 180.0f / CV_PI;
  // Convert to degrees

  // Calculate standard deviation for motion consistency
  float sum_sq_dx = 0;
  float sum_sq_dy = 0;
  for (size_t i = 0; i < dx_values.size(); ++i) {
    sum_sq_dx += (dx_values[i] - motion.mean_dx) * (
      dx_values[i] - motion.mean_dx);
    sum_sq_dy += (dy_values[i] - motion.mean_dy) * (
      dy_values[i] - motion.mean_dy);
  }
  motion.std_dx = sqrt(sum_sq_dx / dx_values.size());
  motion.std_dy = sqrt(sum_sq_dy / dy_values.size());

  // Determine motion type
  if (motion.mean_magnitude < 1.0f) {
    motion.motion_type = MotionSummary::MotionType::kMinimal;
  } else if (motion.std_dx < 2.0f && motion.std_dy < 2.0f) {
    motion.motion_type = MotionSummary::MotionType::kRigid;
  } else {
    motion.motion_type = MotionSummary::MotionType::kDeformative;
  }
  return motion;
}

MotionSummary ComputeMotionSummary(
    const cv::Mat& image_frame, const cv::Mat& previous_image_frame) {
  CHECK(!previous_image_frame.empty()) << "previous_image is empty.";
  CHECK(!image_frame.empty()) << "image_frame is empty.";
  CHECK(previous_image_frame.channels() == 1)
      << "previous_image_frame must be grayscale.";
  CHECK(image_frame.channels() == 1)
      << "image_frame must be grayscale.";

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
      previous_image_frame, // Previous image
      image_frame, // Next image
      corners_a, // Previous set of corners (from imgA)
      corners_b, // Next set of corners (from imgB)
      features_found, // Output vector, each is 1 for tracked
      cv::noArray(), // Output vector, lists errors (optional)
      cv::Size(win_size * 2 + 1, win_size * 2 + 1), // Search window size
      5, // Maximum pyramid level to construct
      cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                       20, // Maximum number of iterations
                       0.3 // Minimum change per iteration
          ));

  return CalculateFromLK(corners_a, corners_b, features_found);
}
} // namespace sfm