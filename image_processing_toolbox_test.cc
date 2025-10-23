#include "image_processing_toolbox.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "tools/cpp/runfiles/runfiles.h"

namespace sfm {
namespace {
using ::absl_testing::IsOk;
using ::bazel::tools::cpp::runfiles::Runfiles;

TEST(SaveFrames, Works) {
  const char* tmp_dir = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp_dir, nullptr) << "TEST_TMPDIR not set";

  const std::string output_path = std::filesystem::path(tmp_dir) / "frames";
  Runfiles* files = Runfiles::CreateForTest();
  ASSERT_THAT(SaveFrames(files->Rlocation("_main/testdata/video_10s.mp4"), 10,
                output_path),
              IsOk());

  int32_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(output_path)) {
    if (entry.is_regular_file()) {
      ++count;
    }
  }
  EXPECT_EQ(count, 10);
}

TEST(ComputeMotionSummary, Works) {
  Runfiles* files = Runfiles::CreateForTest();
  const cv::Mat previous_frame = cv::imread(
      files->Rlocation("_main/testdata/frame_0.jpg"), cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(previous_frame.empty());
  const cv::Mat image_frame = cv::imread(
      files->Rlocation("_main/testdata/frame_27.jpg"), cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image_frame.empty());

  MotionSummary motion_summary = ComputeMotionSummary(
      image_frame, previous_frame);
  EXPECT_THAT(motion_summary.motion_type, MotionSummary::MotionType::kDeformative);
  EXPECT_NEAR(0.92, motion_summary.tracked_points_ratio, 0.1);
  EXPECT_NEAR(31.7, motion_summary.mean_dx, 0.1);
  EXPECT_NEAR(31.7, motion_summary.mean_dy, 0.1);
  EXPECT_NEAR(44.7, motion_summary.mean_magnitude, 0.1);
  EXPECT_NEAR(45.0, motion_summary.mean_direction, 0.1);
  EXPECT_NEAR(40.6, motion_summary.std_dx, 0.1);
  EXPECT_NEAR(91.1, motion_summary.std_dy, 0.1);
}

TEST(ComputeMotionSummary, TheSameWorks) {
  Runfiles* files = Runfiles::CreateForTest();
  const cv::Mat image_frame = cv::imread(
      files->Rlocation("_main/testdata/frame_27.jpg"), cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image_frame.empty());

  MotionSummary motion_summary = ComputeMotionSummary(
      image_frame, image_frame);
  EXPECT_THAT(motion_summary.motion_type, MotionSummary::MotionType::kMinimal);
  EXPECT_NEAR(1., motion_summary.tracked_points_ratio, 0.01);
  EXPECT_NEAR(0., motion_summary.std_dx, 0.01);
  EXPECT_NEAR(0., motion_summary.std_dy, 0.01);
}

TEST(ComputeMotionSummary, CompletelyDifferentWorks) {
  Runfiles* files = Runfiles::CreateForTest();
  // Previous and current frames have nothing in common
  const cv::Mat previous_frame = cv::imread(
      files->Rlocation("_main/testdata/reconstruction/frame_0.jpg"), cv::IMREAD_GRAYSCALE);
  const cv::Mat image_frame = cv::imread(
      files->Rlocation("_main/testdata/frame_27.jpg"), cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image_frame.empty());

  MotionSummary motion_summary = ComputeMotionSummary(
      image_frame, previous_frame);
  EXPECT_NEAR(0, motion_summary.std_dx, 0.01);
  EXPECT_NEAR(0, motion_summary.std_dy, 0.01);
}

} // namespace
} // namespace sfm