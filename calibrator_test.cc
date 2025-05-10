#include "calibrator.h"
#include "absl/status/status_matchers.h"
#include "calibration_data.pb.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "protobuf-matchers/protocol-buffer-matchers.h"
#include "tools/cpp/runfiles/runfiles.h"

namespace sfm {
namespace {

using ::absl_testing::IsOk;
using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using ::bazel::tools::cpp::runfiles::Runfiles;
using ::protobuf_matchers::EqualsProto;
using ::testing::HasSubstr;


TEST(Calibrator, InvalidFailed) {
  EXPECT_THAT(Calibrate("foo"),
              StatusIs(absl::StatusCode::kInvalidArgument, HasSubstr("exist")));
}

TEST(Calibrator, Works) {
  const Runfiles* files = Runfiles::CreateForTest();
  const std::string images_directory =
      files->Rlocation("_main/testdata/calibration");

  EXPECT_THAT(Calibrate(images_directory),
              IsOkAndHolds(EqualsProto(R"pb(reprojection_error: 23)pb")));
}

}  // namespace
}  // namespace sfm