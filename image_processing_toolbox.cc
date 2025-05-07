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
}  // namespace sfm