#include "slam.h"
#include "status_macros.h"

namespace sfm {

SLAM::SLAM(const proto::CameraMatrix& camera_matrix) {
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = camera_matrix.fx();
  camera_matrix_.at<double>(1, 1) = camera_matrix.fy();
  camera_matrix_.at<double>(0, 2) = camera_matrix.cx();
  camera_matrix_.at<double>(1, 2) = camera_matrix.cy();
  const cv::Mat initial_pose = cv::Mat::eye(3, 4, CV_64F);
  initialized_ = false;
  // camera_poses_.p0ush_back(initial_pose.clone());
}

absl::Status SLAM::ProcessFrame(const ImageFrame& image_frame) {
  if (!initialized_) Initialize(image_frame);
  return absl::OkStatus();
}

void SLAM::Initialize(const ImageFrame& image_frame) {
  current_frame_ = image_frame;
  // TODO: Detect features
  initialized_ = true;
}

absl::Status SLAM::ProcessImagePaths(
    const std::vector<std::string>& image_paths) {
  if (image_paths.size() < 2) {
    return absl::InternalError("At least 2 images are required for SfM");
  }

  for (const auto& file_path : image_paths) {
    cv::Mat img = cv::imread((file_path));
    if (img.empty())
      return absl::InternalError(
          absl::StrCat("Failed to load image: ", file_path));
    RETURN_IF_ERROR(ProcessFrame(img));
  }
  return absl::OkStatus();
}

}  // namespace sfm
