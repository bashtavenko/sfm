#include "image_processing_toolbox.h"
#include <filesystem>

#include "absl/strings/str_format.h"
#include "opencv2/highgui.hpp"
#include "absl/log/check.h"
#include "status_macros.h"
#include "absl/log/log.h"

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

absl::StatusOr<std::vector<cv::DMatch>> MatchFeatures(
    const cv::Ptr<cv::Feature2D>& detector,
    const cv::Ptr<cv::DescriptorMatcher>& matcher, const cv::Mat& image,
    const cv::Mat& previous_image,
    float ratio_threshold) {
  CHECK(!image.empty()) << "previous_image is empty.";
  CHECK(!previous_image.empty()) << "image_frame is empty.";
  CHECK(image.channels() == 1)
      << "image must be grayscale.";
  CHECK(previous_image.channels() == 1)
      << "previous_image must be grayscale.";

  // Detect keypoints
  std::vector<cv::KeyPoint> kp_a;
  std::vector<cv::KeyPoint> kp_b;
  cv::Mat des_a;
  cv::Mat des_b;
  detector->detectAndCompute(image, cv::noArray(), kp_a, des_a);
  detector->detectAndCompute(previous_image, cv::noArray(), kp_b, des_b);

  // Match features
  std::vector<std::vector<cv::DMatch>> knn_matches;
  std::vector<cv::DMatch> good_matches;
  matcher->knnMatch(des_a, des_b, knn_matches, /*k=*/2);

  // Apply ratio test
  for (size_t j = 0; j < knn_matches.size(); ++j) {
    if (knn_matches[j].size() >= 2) {
      if (knn_matches[j][0].distance > 0 && knn_matches[j][0].distance <
          ratio_threshold * knn_matches[j][1].distance) {
        good_matches.push_back(knn_matches[j][0]);
      }
    }
  }

  return good_matches;
}

absl::Status SaveRelatedFrames(absl::string_view video_path,
                               absl::string_view output_directory,
                               int32_t minimum_feature_count) {
  std::filesystem::path output_dir(output_directory);
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
  int32_t total_frame_count = static_cast<int32_t>(cap.get(
      cv::CAP_PROP_FRAME_COUNT));
  if (total_frame_count <= 0) {
    return absl::InternalError("Failed to read video frame count.");
  }
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(
      cv::DescriptorMatcher::FLANNBASED);
  size_t frame_count = 0;
  size_t output_frame_count = 0;
  cv::Mat image;
  cv::Mat previous_image;
  while (cap.read(image)) {
    ++frame_count;
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    if (frame_count == 1) {
      previous_image = image;
      continue;
    }
    // Actually match the features
    ASSIGN_OR_RETURN(auto good_fetures,
                     MatchFeatures(detector, matcher, image, previous_image));
    if (good_fetures.size() > minimum_feature_count) {
      ++output_frame_count;
      std::ostringstream filename;
      filename << absl::StreamFormat("frame_%i.jpg", frame_count);
      std::filesystem::path output_file = output_dir / filename.str();
      if (!cv::imwrite(output_file.string(), image)) {
        return absl::InternalError(absl::StrFormat(
            "Failed to save frame to '%s'.", output_file.string()));
      }
    }
    previous_image = image;
  }
  LOG(INFO) << absl::StreamFormat("Saved %i out of %i frames",
                                  output_frame_count, total_frame_count);

  return absl::OkStatus();
}
} // namespace sfm