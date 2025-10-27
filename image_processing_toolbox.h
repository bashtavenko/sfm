#ifndef IMAGE_PROCESSING_TOOLBOX_H
#define IMAGE_PROCESSING_TOOLBOX_H
#include "absl/status/statusor.h"

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "opencv2/opencv.hpp"

namespace sfm {
// Saves k frames from the given video into output directory
absl::Status SaveFrames(absl::string_view video_path, int32_t k,
                        absl::string_view frame_output_directory);

// For the given video selects frames that overlaps with previous frame and
// have enough new features.
// minimum_feature_count is the lower bound for the matching new frame features.
// Frames are saved when actual feature count is higher than that parameter.
absl::Status SaveRelatedFrames(absl::string_view video_path,
                               absl::string_view output_directory,
                               size_t minimum_feature_count = 20);

// Computes SIFT descriptor for the given image.
cv::Mat ComputeDescriptor(const cv::Ptr<cv::Feature2D>& detector,
                          const cv::Mat& image);

// Returns matches of two SIFT descriptors
absl::StatusOr<std::vector<cv::DMatch>> MatchFeatures(
    const cv::Ptr<cv::DescriptorMatcher>& matcher, const cv::Mat& descriptor_a,
    const cv::Mat& descriptor_b);
} // namespace sfm

#endif  // IMAGE_PROCESSING_TOOLBOX_H
