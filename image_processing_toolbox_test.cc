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
}  // namespace
}  // namespace sfm