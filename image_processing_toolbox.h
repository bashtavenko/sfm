#ifndef IMAGE_PROCESSING_TOOLBOX_H
#define IMAGE_PROCESSING_TOOLBOX_H
#include <filesystem>
#include "absl/status/statusor.h"
#include "opencv2/features2d.hpp"

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace sfm {

// Saves k frames from the given video into output directory
absl::Status SaveFrames(absl::string_view video_path, int32_t k,
                        absl::string_view frame_output_directory);

// Returns SIFT matches for two images and given threshold.
// The number of return matches varies from 0 to 50-100. The same image
// returns zero matches.
absl::StatusOr<std::vector<cv::DMatch>> MatchFeatures(const cv::Ptr<cv::Feature2D>& detector,
  const cv::Ptr<cv::DescriptorMatcher>& matcher, const cv::Mat& image,
  const cv::Mat& previous_image,
  float ratio_threshold = 0.75f);


}  // namespace sfm

#endif  // IMAGE_PROCESSING_TOOLBOX_H
