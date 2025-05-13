#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <vector>
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "opencv2/core.hpp"
#include "calibration_data.pb.h"

namespace sfm {

class Reconstruction {
 public:
  // Initialize camera intrinsict and initial camera pose.
  Reconstruction(const proto::CameraMatrix& camera_matrix);
  absl::Status Run(const std::vector<std::string>& image_paths,
    absl::string_view file_path);
  cv::Mat GetPointColors() const;
  std::vector<cv::Mat> GetCameraPoses() const;
  cv::Mat GetPointCloud() const;

 private:
  enum class DescriptorType {
    kSift,
    kOrb,
  };
  absl::Status LoadImages(const std::vector<std::string>& image_paths);
  absl::StatusOr<std::vector<int32_t>> DetectFeatures(
      DescriptorType descriptor_type = DescriptorType::kSift);
  std::vector<int32_t> MatchFeatures(float ratio_threshold = 0.75f);
  absl::Status EstimateCameraPoses();
  absl::Status TriangulatePoints();
  absl::Status SavePointCloud(absl::string_view file_path);
  cv::Mat camera_matrix_;
  std::vector<cv::Mat> images_;
  std::vector<cv::Mat> camera_poses_;
  std::vector<std::vector<cv::KeyPoint>> keypoints_;
  std::vector<cv::Mat> descriptors_;
  std::vector<std::vector<cv::DMatch>> feature_matches_;
  cv::Mat point_cloud_;
  cv::Mat point_colors_;
};

}  // namespace sfm

#endif  // RECONSTRUCTION_H
