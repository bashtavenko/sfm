#include "image_processing_toolbox.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "tools/cpp/runfiles/runfiles.h"

namespace sfm {
namespace {
using ::absl_testing::IsOk;
using ::absl_testing::IsOkAndHolds;
using ::testing::SizeIs;
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

TEST(MatchFeatures, TwoValidImages) {
  Runfiles* files = Runfiles::CreateForTest();
  const cv::Mat image = cv::imread(
      files->Rlocation(
          "_main/testdata/reconstruction/kleenex/two_frames/frame_1.jpg"),
      cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty());
  const cv::Mat previous_image = cv::imread(
      files->Rlocation(
          "_main/testdata/reconstruction/kleenex/two_frames/frame_35.jpg"),
      cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(previous_image.empty());
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(
      cv::DescriptorMatcher::FLANNBASED);

  auto matches = MatchFeatures(detector, matcher, image, previous_image);
  EXPECT_THAT(matches, IsOkAndHolds(SizeIs(70)));
}

TEST(MatchFeatures, SameImage) {
  Runfiles* files = Runfiles::CreateForTest();
  const cv::Mat image = cv::imread(
      files->Rlocation(
          "_main/testdata/reconstruction/kleenex/two_frames/frame_1.jpg"),
      cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty());
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(
      cv::DescriptorMatcher::FLANNBASED);

  auto matches = MatchFeatures(detector, matcher, image, image);
  EXPECT_THAT(matches, IsOkAndHolds(SizeIs(0)));
}

TEST(MatchFeatures, BogusImage) {
  Runfiles* files = Runfiles::CreateForTest();
  const cv::Mat image = cv::imread(
      files->Rlocation(
          "_main/testdata/reconstruction/kleenex/two_frames/frame_1.jpg"),
      cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty());
  const cv::Mat previous_image = cv::imread(
      files->Rlocation(
          "_main/testdata/reconstruction/frame_0.jpg"),
      cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(previous_image.empty());
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(
      cv::DescriptorMatcher::FLANNBASED);

  auto matches = MatchFeatures(detector, matcher, image, previous_image);
  // Not even close
  EXPECT_THAT(matches, IsOkAndHolds(SizeIs(15)));
}
} // namespace
} // namespace sfm