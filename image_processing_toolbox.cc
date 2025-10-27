#include "image_processing_toolbox.h"
#include <filesystem>

#include "absl/strings/str_format.h"
#include "status_macros.h"
#include "glog/logging.h"
#include "opencv2/features2d.hpp"
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

cv::Mat ComputeDescriptor(const cv::Ptr<cv::Feature2D>& detector,
                          const cv::Mat& image) {
  cv::Mat descriptor;
  constexpr float kScaleFactor = 0.5f;
  CHECK(!image.empty()) << "Empty image.";
  cv::Mat converted;
  cv::cvtColor(image, converted, cv::COLOR_BGR2GRAY);
  cv::resize(converted, converted, cv::Size(), kScaleFactor, kScaleFactor,
             cv::INTER_LINEAR);
  std::vector<cv::KeyPoint> keypoints;
  detector->detectAndCompute(converted, cv::noArray(), keypoints, descriptor);
  return descriptor;
}

absl::StatusOr<std::vector<cv::DMatch>> MatchFeatures(
    const cv::Ptr<cv::DescriptorMatcher>& matcher, const cv::Mat& descriptor_a,
    const cv::Mat& descriptor_b) {
  constexpr float kRatioThreshold = 0.75f;
  // Match features
  std::vector<std::vector<cv::DMatch>> knn_matches;
  std::vector<cv::DMatch> good_matches;
  matcher->knnMatch(descriptor_a, descriptor_b, knn_matches, /*k=*/2);

  // Apply ratio test
  for (size_t j = 0; j < knn_matches.size(); ++j) {
    // LOG(INFO) << absl::StreamFormat("Distance %f : %f",
    //                                 knn_matches[j][0].distance,
    //                                 knn_matches[j][1].distance);
    if (knn_matches[j].size() >= 2) {
      if (knn_matches[j][0].distance > 0 && knn_matches[j][0].distance <
          kRatioThreshold * knn_matches[j][1].distance) {
        good_matches.push_back(knn_matches[j][0]);
      }
    }
  }

  return good_matches;
}

absl::Status SaveRelatedFrames(absl::string_view video_path,
                               absl::string_view output_directory,
                               size_t minimum_feature_count) {
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
  cv::Mat image_descriptor;
  cv::Mat previous_descriptor;
  LOG(INFO) << "Frame count: " << total_frame_count;
  while (cap.read(image)) {
    ++frame_count;
    if (frame_count == 1) {
      previous_image = image.clone();
      previous_descriptor = ComputeDescriptor(detector, previous_image);
      continue;
    }
    image_descriptor = ComputeDescriptor(detector, image);
    // Actually match the features
    ASSIGN_OR_RETURN(auto good_fetures,
                     MatchFeatures(matcher, image_descriptor,
                       previous_descriptor));
    if (good_fetures.size() > minimum_feature_count) {
      ++output_frame_count;
      std::ostringstream filename;
      filename << absl::StreamFormat("frame_%i.jpg", output_frame_count);
      std::filesystem::path output_file = output_dir / filename.str();
      if (!cv::imwrite(output_file.string(), image)) {
        return absl::InternalError(absl::StrFormat(
            "Failed to save frame to '%s'.", output_file.string()));
      }
      previous_image = image.clone();
      previous_descriptor.copyTo(image_descriptor);
    }
  }
  LOG(INFO) << absl::StreamFormat("Saved %i out of %i frames",
                                  output_frame_count, total_frame_count);

  return absl::OkStatus();
}
} // namespace sfm