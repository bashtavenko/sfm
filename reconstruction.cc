#include "reconstruction.h"
#include <fstream>
#include "absl/strings/str_format.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"
#include "sfm.h"
#include "status_macros.h"
#include "glog/logging.h"

namespace sfm {
Reconstruction::Reconstruction(const proto::CameraMatrix& camera_matrix) {
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = camera_matrix.fx();
  camera_matrix_.at<double>(1, 1) = camera_matrix.fy();
  camera_matrix_.at<double>(0, 2) = camera_matrix.cx();
  camera_matrix_.at<double>(1, 2) = camera_matrix.cy();
  const cv::Mat initial_pose = cv::Mat::eye(3, 4, CV_64F);
  camera_poses_.push_back(initial_pose.clone());
}

absl::Status Reconstruction::Run(const std::vector<std::string>& image_paths,
                                 absl::string_view point_cloud_path) {
  RETURN_IF_ERROR(LoadImages(image_paths));
  ASSIGN_OR_RETURN(auto features, DetectFeatures());
  for (const auto& feature : features) {
    LOG(INFO) << "Detected feature points: " << feature;
  }
  LOG(INFO) << "Matching features between consecutive images...";
  std::vector<int32_t> features_match = MatchFeatures();
  for (const auto& match : features_match) {
    LOG(INFO) << "Feature matches: " << match;
  }
  LOG(INFO) << "Estimating camera poses...";
  RETURN_IF_ERROR(EstimateCameraPoses());
  LOG(INFO) << absl::StreamFormat("Estimated %i camera poses.",
                                  GetCameraPoses().size());
  LOG(INFO) << "Triangulating points...";
  RETURN_IF_ERROR(TriangulatePoints());
  LOG(INFO) << absl::StreamFormat("Triangulated points: %i",
                                  GetPointCloud().dims);
  LOG(INFO) << absl::StreamFormat("Saving 3D point cloud...");
  RETURN_IF_ERROR(SavePointCloud(point_cloud_path));
  LOG(INFO) << "Done.";
  return absl::OkStatus();
}

cv::Mat Reconstruction::GetPointCloud() const { return point_cloud_; }

cv::Mat Reconstruction::GetPointColors() const { return point_colors_; }

std::vector<cv::Mat> Reconstruction::GetCameraPoses() const {
  return camera_poses_;
}

absl::Status Reconstruction::LoadImages(
    const std::vector<std::string>& image_paths) {
  images_.clear();

  for (size_t i = 0; i < image_paths.size(); ++i) {
    cv::Mat img = cv::imread(image_paths[i]);
    if (!img.empty()) {
      images_.push_back(img);
    }
  }

  if (images_.size() < 2) {
    return absl::InternalError("At least 2 images are required for SfM");
  }
  return absl::OkStatus();
}

absl::StatusOr<std::vector<int32_t>> Reconstruction::DetectFeatures(
    DescriptorType descriptor_type) {
  keypoints_.clear();
  descriptors_.clear();
  std::vector<int32_t> feature_counts;

  cv::Ptr<cv::Feature2D> detector;
  switch (descriptor_type) {
    case DescriptorType::kSift:
      detector = cv::SIFT::create();
      break;
    case DescriptorType::kOrb:
      detector = cv::ORB::create();
      break;
    default:
      return absl::InternalError("Unknown descriptor type");
  }

  for (size_t i = 0; i < images_.size(); ++i) {
    std::vector<cv::KeyPoint> kp;
    cv::Mat des;
    cv::Mat gray;

    cv::cvtColor(images_[i], gray, cv::COLOR_BGR2GRAY);
    detector->detectAndCompute(gray, cv::noArray(), kp, des);

    keypoints_.push_back(kp);
    descriptors_.push_back(des);
    feature_counts.push_back(static_cast<int>(kp.size()));
  }

  return feature_counts;
}

std::vector<int32_t> Reconstruction::MatchFeatures(float ratio_threshold) {
  feature_matches_.clear();
  std::vector<int32_t> match_counts;
  static constexpr int32_t kMinGoodMatchesSize = 8;

  for (size_t i = 0; i < images_.size() - 1; ++i) {
    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::DMatch> good_matches;

    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (descriptors_[0].type() == CV_32F) {
      matcher =
          cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    } else {
      matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
    }

    matcher->knnMatch(descriptors_[i], descriptors_[i + 1], knn_matches, 2);

    // Apply ratio test
    for (size_t j = 0; j < knn_matches.size(); ++j) {
      if (knn_matches[j].size() >= 2) {
        if (knn_matches[j][0].distance <
            ratio_threshold * knn_matches[j][1].distance) {
          good_matches.push_back(knn_matches[j][0]);
        }
      }
    }

    if (good_matches.size() < kMinGoodMatchesSize) {
      LOG(INFO) << "Skipping too few correspondences: " << good_matches.size();
      continue;
    }
    feature_matches_.push_back(good_matches);
    match_counts.push_back(static_cast<int>(good_matches.size()));
  }
  return match_counts;
}

absl::Status Reconstruction::EstimateCameraPoses() {
  for (size_t i = 0; i < feature_matches_.size(); ++i) {
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;

    // Get matched keypoints
    for (size_t j = 0; j < feature_matches_[i].size(); ++j) {
      pts1.push_back(keypoints_[i][feature_matches_[i][j].queryIdx].pt);
      pts2.push_back(keypoints_[i + 1][feature_matches_[i][j].trainIdx].pt);
    }

    // Estimate essential matrix. It is 3 x 3
    // E = [t]x R. Where [t]x - skewed symmetric matrix of translation vector (3x1)
    // R = rotation matrix (3 x 3)
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, camera_matrix_, cv::RANSAC,
                                     0.999, 1.0, mask);

    // Recover pose (rotation and translation)
    cv::Mat R;
    cv::Mat t;

    CHECK(E.isContinuous());
    CHECK(mask.isContinuous());
    CHECK(camera_matrix_.isContinuous());
    cv::recoverPose(E, pts1, pts2, camera_matrix_, R, t, mask);

    // Get previous pose
    cv::Mat prev_R = camera_poses_.back()(cv::Rect(0, 0, 3, 3));
    cv::Mat prev_t = camera_poses_.back()(cv::Rect(3, 0, 1, 3));

    // Compute current absolute pose
    cv::Mat curr_R = R * prev_R;
    cv::Mat curr_t = R * prev_t + t;

    // Create new camera pose [R|t]
    cv::Mat pose = cv::Mat::eye(3, 4, CV_64F);
    curr_R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
    curr_t.copyTo(pose(cv::Rect(3, 0, 1, 3)));

    camera_poses_.push_back(pose);
  }
  return absl::OkStatus();
}

absl::Status Reconstruction::TriangulatePoints() {
  std::vector<cv::Point3f> all_points_3d;
  std::vector<cv::Vec3b> all_point_colors;

  for (size_t i = 0; i < feature_matches_.size(); ++i) {
    // Get camera projection matrices
    cv::Mat P1 = camera_matrix_ * camera_poses_[i];
    cv::Mat P2 = camera_matrix_ * camera_poses_[i + 1];

    // Get matched keypoints
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;

    for (size_t j = 0; j < feature_matches_[i].size(); ++j) {
      pts1.push_back(keypoints_[i][feature_matches_[i][j].queryIdx].pt);
      pts2.push_back(keypoints_[i + 1][feature_matches_[i][j].trainIdx].pt);
    }

    // Triangulate points
    cv::Mat points_4d;
    cv::triangulatePoints(P1, P2, pts1, pts2, points_4d);

    // Convert to 3D
    for (int j = 0; j < points_4d.cols; ++j) {
      cv::Vec4f point = points_4d.col(j);
      point /= point[3];  // Normalize by w

      cv::Point3f point_3d(point[0], point[1], point[2]);
      all_points_3d.push_back(point_3d);

      // Get color from first image
      int x = static_cast<int>(pts1[j].x);
      int y = static_cast<int>(pts1[j].y);

      cv::Vec3b color;
      if (x >= 0 && x < images_[i].cols && y >= 0 && y < images_[i].rows) {
        color = images_[i].at<cv::Vec3b>(y, x);
      } else {
        color = cv::Vec3b(0, 0, 0);
      }

      all_point_colors.push_back(color);
    }
  }

  // Convert vectors to cv::Mat
  point_cloud_ = cv::Mat(all_points_3d.size(), 3, CV_32F);
  point_colors_ = cv::Mat(all_point_colors.size(), 3, CV_8UC1);

  for (size_t i = 0; i < all_points_3d.size(); ++i) {
    point_cloud_.at<float>(i, 0) = all_points_3d[i].x;
    point_cloud_.at<float>(i, 1) = all_points_3d[i].y;
    point_cloud_.at<float>(i, 2) = all_points_3d[i].z;

    point_colors_.at<uchar>(i, 0) = all_point_colors[i][0];
    point_colors_.at<uchar>(i, 1) = all_point_colors[i][1];
    point_colors_.at<uchar>(i, 2) = all_point_colors[i][2];
  }
  return absl::OkStatus();
}

absl::Status Reconstruction::SavePointCloud(absl::string_view file_path) {
  if (point_cloud_.empty())
    return absl::InternalError("No point cloud to save.");

  std::ofstream file(file_path.data());
  if (!file.is_open()) {
    return absl::InternalError(
        absl::StrFormat("Could not open file for writing: %s", file_path));
  }

  // Write PLY header
  file << "ply" << std::endl;
  file << "format ascii 1.0" << std::endl;
  file << "element vertex " << point_cloud_.rows << std::endl;
  file << "property float x" << std::endl;
  file << "property float y" << std::endl;
  file << "property float z" << std::endl;
  file << "property uchar red" << std::endl;
  file << "property uchar green" << std::endl;
  file << "property uchar blue" << std::endl;
  file << "end_header" << std::endl;

  // Write vertices
  for (int i = 0; i < point_cloud_.rows; ++i) {
    file << point_cloud_.at<float>(i, 0) << " " << point_cloud_.at<float>(i, 1)
         << " " << point_cloud_.at<float>(i, 2) << " "
         << static_cast<int>(point_colors_.at<uchar>(i, 2)) << " "         // R
         << static_cast<int>(point_colors_.at<uchar>(i, 1)) << " "         // G
         << static_cast<int>(point_colors_.at<uchar>(i, 0)) << std::endl;  // B
  }

  file.close();
  return absl::OkStatus();
}

absl::Status Reconstruction::VisualizeMatches(int image_idx1, int image_idx2) const {
  if (image_idx1 >= images_.size() || image_idx2 >= images_.size()) {
    return absl::InvalidArgumentError("Invalid image indices.");
  }

  if (image_idx1 >= feature_matches_.size()) {
    return absl::InvalidArgumentError("Feature matches not available for image_idx1.");
  }

  const auto& img1 = images_[image_idx1];
  const auto& img2 = images_[image_idx2];
  const auto& kpts1 = keypoints_[image_idx1];
  const auto& kpts2 = keypoints_[image_idx2];
  const auto& matches = feature_matches_[image_idx1];  // Assuming pairwise [i] = match between i and i+1

  // Create masks for unmatched keypoints
  std::vector<char> matched1(kpts1.size(), 0);
  std::vector<char> matched2(kpts2.size(), 0);
  for (const auto& m : matches) {
    matched1[m.queryIdx] = 1;
    matched2[m.trainIdx] = 1;
  }

  // Convert to color images for drawing
  cv::Mat color_img1, color_img2;
  if (img1.channels() == 1) cv::cvtColor(img1, color_img1, cv::COLOR_GRAY2BGR);
  else img1.copyTo(color_img1);
  if (img2.channels() == 1) cv::cvtColor(img2, color_img2, cv::COLOR_GRAY2BGR);
  else img2.copyTo(color_img2);

  // Draw unmatched keypoints in black
  for (size_t i = 0; i < kpts1.size(); ++i) {
    if (!matched1[i]) {
      cv::circle(color_img1, kpts1[i].pt, 3, cv::Scalar(0, 0, 0), -1);
    }
  }
  for (size_t i = 0; i < kpts2.size(); ++i) {
    if (!matched2[i]) {
      cv::circle(color_img2, kpts2[i].pt, 3, cv::Scalar(0, 0, 0), -1);
    }
  }

  // Draw matches in white
  cv::Mat match_vis;
  cv::drawMatches(
      color_img1, kpts1,
      color_img2, kpts2,
      matches, match_vis,
      cv::Scalar(255, 255, 255),  // match color (white)
      cv::Scalar::all(-1),        // single point color
      {},                         // match mask
      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  // Find and draw homography inliers
  if (matches.size() >= 4) {
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : matches) {
      pts1.push_back(kpts1[m.queryIdx].pt);
      pts2.push_back(kpts2[m.trainIdx].pt);
    }
    std::vector<uchar> inliers_mask;
    cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC, 3.0, inliers_mask);
    if (!H.empty()) {
      // Optionally draw inlier matches only
      // Could also overlay bounding boxes if useful
    }
  }

  cv::imshow("Feature Matches", match_vis);
  cv::waitKey(0);
  return absl::OkStatus();
}

}  // namespace sfm