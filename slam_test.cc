#include "slam.h"
#include "absl/status/status_matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "sfm.h"
#include "tools/cpp/runfiles/runfiles.h"

namespace sfm {
namespace {

using ::absl_testing::IsOk;
using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using ::bazel::tools::cpp::runfiles::Runfiles;

TEST(Slam, InitWorks) {
  const Runfiles* files = Runfiles::CreateForTest();
  const std::string text_proto_file_path =
      files->Rlocation("_main/testdata/pixel_6a_calibration.txtpb");
  auto camera_matrix = LoadCameraMatrixFromTextProtoFile(text_proto_file_path);
  EXPECT_THAT(camera_matrix, IsOk());
  auto slam = SLAM(camera_matrix.value());

  const std::string images_directory =
      files->Rlocation("_main/testdata/reconstruction/kleenex/two_frames");
  auto image_paths = GetFilesFromDirectory(images_directory);
  EXPECT_THAT(image_paths, IsOk());
  EXPECT_THAT(slam.ProcessImagePaths(image_paths.value()), IsOk());
}

}  // namespace
}  // namespace sfm