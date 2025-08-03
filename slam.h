//
// Created by Stan Bashtavenko on 8/3/25.
//

#ifndef SLAM_H
#define SLAM_H

// class SLAM {
// public:
//   // Main SLAM processing loop - processes one frame at a time
//   absl::Status ProcessFrame(const std::string& image_path);
//
//   // Initialize with first frame
//   absl::Status Initialize(const std::string& first_image_path);
//
//   // Save current map
//   absl::Status SaveMap(absl::string_view point_cloud_path);
//
//   // Get current camera pose
//   const CameraPose& GetCurrentPose() const { return current_pose_; }
//
//   // Get current map
//   const PointCloud& GetMap() const { return global_map_; }
//
// private:
//   // Core processing functions (adapted from your SfM)
//   absl::Status LoadCurrentFrame(const std::string& image_path);
//   absl::StatusOr<Features> DetectFeaturesCurrentFrame();
//   std::vector<int32_t> MatchWithPreviousFrame();
//   std::vector<int32_t> MatchWithKeyframes(); // For loop closure
//   absl::Status EstimateCurrentPose();
//   absl::Status UpdateMap();
//   absl::Status CheckForKeyframe();
//   absl::Status DetectLoopClosure();
//
//   // SLAM-specific state
//   cv::Mat current_frame_;
//   cv::Mat previous_frame_;
//   Features current_features_;
//   Features previous_features_;
//
//   CameraPose current_pose_;
//   CameraPose previous_pose_;
//
//   // Keyframe management
//   std::deque<Keyframe> keyframes_;
//   static constexpr size_t MAX_KEYFRAMES = 10; // Keep recent keyframes
//
//   // Global map
//   PointCloud global_map_;
//
//   // Simple loop closure
//   std::vector<Features> keyframe_features_;
//   std::vector<CameraPose> keyframe_poses_;
//
//   bool initialized_ = false;
// };

#endif //SLAM_H
