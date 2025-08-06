#include "calibrator.h"
#include <filesystem>
#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

namespace sfm {

// This is a regular "9x6 OpenCV chessboard" in portrait mode.
// These dimensions refer to the number of inner corners and
// not the number of squares.S
// 6 internal corners, 9 internal corners
// 9 columns and 6 rows
static constexpr int32_t kBoardWidth = 6;
static constexpr int32_t kBoardHeight = 9;

absl::StatusOr<sfm::proto::CalibrationResult> Calibrate(
    absl::string_view images_directory) {
  if (!std::filesystem::exists(images_directory)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Images directory '%s' does not exist.", images_directory));
  }

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  const cv::Size board(kBoardWidth, kBoardHeight);
  std::vector<cv::Point3f> obj;
  for (int i = 0; i < kBoardHeight; ++i) {
    for (int j = 0; j < kBoardWidth; ++j) {
      obj.emplace_back(j, i, 0.0f);
    }
  }

  cv::Size image_size;
  for (const auto& entry :
       std::filesystem::directory_iterator(images_directory)) {
    cv::Mat image = cv::imread(entry.path(), cv::IMREAD_GRAYSCALE);
    if (!image.data) {
      LOG(INFO) << absl::StreamFormat("File '%s' is not an image.",
                                      entry.path());
      continue;
    }

    std::vector<cv::Point2f> corners;
    if (!cv::findChessboardCorners(image, board, corners)) {
      LOG(INFO) << absl::StreamFormat(
          "Could not find chessboard corners in '%s'", entry.path());
      continue;
    }

    // Refine corner locations for better precision
    // 11 x 11 means half-width and half-height search area
    // Chessboards printed with square sizes >= 20-25 pixels on the image
    cv::cornerSubPix(
        image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
                         0.1));

    image_points.push_back(corners);
    object_points.push_back(obj);
    image_size = image.size();
  }

  cv::Mat camera_matrix;
  cv::Mat distortions;
  const double reprojection_error = cv::calibrateCamera(
      object_points, image_points, image_size, camera_matrix, distortions,
      cv::noArray(), cv::noArray());

  proto::CalibrationResult result;
  // Camera matrix
  result.mutable_camera_matrix()->set_fx(camera_matrix.at<double>(0, 0));
  result.mutable_camera_matrix()->set_cx(camera_matrix.at<double>(0, 2));
  result.mutable_camera_matrix()->set_fy(camera_matrix.at<double>(1, 1));
  result.mutable_camera_matrix()->set_cy(camera_matrix.at<double>(1, 2));
  result.set_reprojection_error(reprojection_error);
  // Distortions
  result.mutable_distortion_parameters()->set_k1(distortions.at<double>(0, 0));
  result.mutable_distortion_parameters()->set_k2(distortions.at<double>(0, 1));
  result.mutable_distortion_parameters()->set_k3(distortions.at<double>(0, 2));
  result.mutable_distortion_parameters()->set_k4(distortions.at<double>(0, 3));
  if (distortions.cols >= 5) {
    result.mutable_distortion_parameters()->set_k5(
        distortions.at<double>(0, 4));
  }
  if (distortions.cols >= 6) {
    result.mutable_distortion_parameters()->set_p1(
        distortions.at<double>(0, 5));
  }
  if (distortions.cols >= 7) {
    result.mutable_distortion_parameters()->set_p2(
        distortions.at<double>(0, 6));
  }

  return result;
}

}  // namespace sfm
