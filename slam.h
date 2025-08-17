#ifndef SLAM_H
#define SLAM_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "calibration_data.pb.h"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"

namespace sfm {

using Keypoints = std::vector<std::vector<cv::KeyPoint>>;
using Descriptors = std::vector<cv::Mat>;
using CameraPose = cv::Mat;
using PointCloud = cv::Mat;
using ImageFrame = cv::Mat;


class SLAM {
 public:
  SLAM(const proto::CameraMatrix& camera_matrix);

  // Online processing
  absl::Status ProcessFrame(const ImageFrame& image_frame);

  // Batch processing
  absl::Status ProcessImagePaths(
      const std::vector<std::string>& image_paths);

  // Save current map
  absl::Status SaveMap(absl::string_view point_cloud_path);

  // Get current camera pose
  const CameraPose& GetCurrentPose() const { return current_pose_; }

  // Get current map
  const PointCloud& GetMap() const { return global_map_; }

 private:
  std::deque<Keypoints> keyframes_;
  static constexpr size_t MAX_KEYFRAMES = 10; // Keep recent keyframes

  // Initialize with first frame
  absl::Status Initialize(const ImageFrame& image_frame);

  // Core processing functions (adapted from your SfM)
  absl::Status LoadCurrentFrame(const std::string& image_path);
  // Detect current keypoints and descriptors
  absl::Status DetectCurrentFrame(const ImageFrame& image_frame);
  absl::Status MatchWithPreviousFrame(float ratio_threshold = 0.75f);
  std::vector<int32_t> MatchWithKeyframes();  // For loop closure
  absl::Status EstimateCurrentPose();
  absl::Status UpdateMap();
  absl::Status CheckForKeyframe();
  absl::Status DetectLoopClosure();

  // SLAM-specific state
  cv::Mat current_frame_;
  cv::Mat previous_frame_;
  Keypoints current_keypoints_;
  Keypoints previous_keypoints_;
  Descriptors current_descriptors_;
  Descriptors previous_descriptors_;
  std::vector<std::vector<cv::DMatch>> feature_matches_;

  CameraPose current_pose_;
  CameraPose previous_pose_;

  cv::Mat management_;
  std::deque<ImageFrame> image_frames_;
  static constexpr size_t MAX_IMAGE_FRAMES = 10;

  // Global map
  cv::Mat global_map_;

  // Simple loop closure
  // Features image_frame_features_;
  std::vector<CameraPose> image_frame_poses_;

  bool initialized_;
  cv::Mat camera_matrix_;
};
}  // namespace sfm

#endif  // SLAM_H
