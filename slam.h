#ifndef SLAM_H
#define SLAM_H

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"

namespace sfm {

using Features = std::vector<std::vector<cv::DMatch>>;
using CameraPose = cv::Mat;
using PointCloud = cv::Mat;
using ImageFrame = cv::Mat;

class SLAM {
public:
  // Main SLAM processing loop - processes one frame at a time
  absl::Status ProcessFrame(const std::string& image_path);

  // Initialize with first frame
  absl::Status Initialize(const std::string& first_image_path);

  // Save current map
  absl::Status SaveMap(absl::string_view point_cloud_path);

  // Get current camera pose
  const CameraPose& GetCurrentPose() const { return current_pose_; }

  // Get current map
  const PointCloud& GetMap() const { return global_map_; }

private:
  // Core processing functions (adapted from your SfM)
  absl::Status LoadCurrentFrame(const std::string& image_path);
  absl::StatusOr<Features> DetectFeaturesCurrentFrame();
  std::vector<int32_t> MatchWithPreviousFrame();
  std::vector<int32_t> MatchWithKeyframes(); // For loop closure
  absl::Status EstimateCurrentPose();
  absl::Status UpdateMap();
  absl::Status CheckForKeyframe();
  absl::Status DetectLoopClosure();

  // SLAM-specific state
  cv::Mat current_frame_;
  cv::Mat previous_frame_;
  Features current_features_;
  Features previous_features_;

  cv::Mat current_pose_;
  cv::Mat previous_pose_;

  cv::Mat management_;
  std::deque<ImageFrame> image_frames_;
  static constexpr size_t MAX_IMAGE_FRAMES = 10;

  // Global map
  cv::Mat global_map_;

  // Simple loop closure
  Features image_frame_features_;
  std::vector<CameraPose> image_frame_poses_;

  bool initialized_ = false;
};
} // namespace sfm

#endif //SLAM_H
