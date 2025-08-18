#include "slam.h"
#include "glog/logging.h"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"
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
  if (!initialized_) {
    previous_frame_ = image_frame;
    RETURN_IF_ERROR(Initialize(image_frame));
  }
  current_frame_ = image_frame;
  RETURN_IF_ERROR(DetectFrame(image_frame, /*is_current*/ true));
  RETURN_IF_ERROR(MatchWithPreviousFrame());
  RETURN_IF_ERROR(EstimateCurrentPose());
  previous_frame_ = image_frame;
  return absl::OkStatus();
}

absl::Status SLAM::Initialize(const ImageFrame& image_frame) {
  RETURN_IF_ERROR(DetectFrame(image_frame, /*is_current=*/false));
  initialized_ = true;
  return absl::OkStatus();
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

absl::Status SLAM::DetectFrame(const ImageFrame& image_frame,
                                      bool is_current) {
  // For simplicity, just use one keypoint detector and initialize on each
  // frame.
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  cv::Mat gray_frame;
  cv::cvtColor(image_frame, gray_frame, cv::COLOR_BGR2GRAY);
  detector->detectAndCompute(gray_frame, /*mask=*/cv::noArray(), keypoints,
                             descriptors);
  if (is_current) {
    current_keypoints_.push_back(keypoints);
    current_descriptors_.push_back(descriptors);
  } else {
    previous_keypoints_.push_back(keypoints);
    previous_descriptors_.push_back(descriptors);
  }
  return absl::OkStatus();
}

absl::Status SLAM::MatchWithPreviousFrame(float ratio_threshold) {
  CHECK(!previous_descriptors_.empty());
  CHECK(!current_descriptors_.empty());
  std::vector<std::vector<cv::DMatch>> knn_matches;
  std::vector<cv::DMatch> good_matches;
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  matcher->knnMatch(previous_descriptors_, current_descriptors_, knn_matches,
                    2);

  // Apply ratio test
  for (size_t j = 0; j < knn_matches.size(); ++j) {
    if (knn_matches[j].size() >= 2) {
      if (knn_matches[j][0].distance <
          ratio_threshold * knn_matches[j][1].distance) {
        good_matches.push_back(knn_matches[j][0]);
      }
    }
  }
  feature_matches_.push_back(good_matches);
  return absl::OkStatus();
}

absl::Status SLAM::EstimateCurrentPose() {
  for (size_t i = 0; i < feature_matches_.size(); ++i) {
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;

    // Get matched keypoints
    for (size_t j = 0; j < feature_matches_.size(); ++j) {
      pts1.push_back(current_keypoints_[i][feature_matches_[i][j].queryIdx].pt);
      pts2.push_back(
          current_keypoints_[i + 1][feature_matches_[i][j].queryIdx].pt);
    }

    // Estimate essential matrix. It is 3 x 3
    // E = [t]x R. Where [t]x - skewed symmetric matrix of translation vector
    // (3x1) R = rotation matrix (3 x 3)
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, camera_matrix_, cv::RANSAC,
                                     0.999, 1.0, mask);

    // Recover pose (rotation and translation)
    cv::Mat R;
    cv::Mat t;

    CHECK(E.isContinuous());
    CHECK(mask.isContinuous());
    CHECK(camera_matrix_.isContinuous());

    // Get previous pose
    cv::Mat prev_R = previous_pose_(cv::Rect(0, 0, 3, 3));
    cv::Mat prev_t = previous_pose_(cv::Rect(3, 0, 1, 3));

    // Compute current absolute pose
    cv::Mat curr_R = R * prev_R;
    cv::Mat curr_t = R * prev_t + t;

    // Create new camera pose [R|t]
    cv::Mat pose = cv::Mat::eye(3, 4, CV_64F);
    curr_R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
    curr_t.copyTo(pose(cv::Rect(3, 0, 1, 3)));
    current_pose_ = pose;
  }
  return absl::OkStatus();
}

}  // namespace sfm
