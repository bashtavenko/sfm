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
using ::protobuf_matchers::Approximately;
using ::protobuf_matchers::EqualsProto;
using ::protobuf_matchers::Partially;
using ::testing::HasSubstr;

constexpr float kTolerance = 1e-1;

TEST(Calibrator, InvalidFailed) {
  EXPECT_THAT(Calibrate("foo"),
              StatusIs(absl::StatusCode::kInvalidArgument, HasSubstr("exist")));
}

TEST(Calibrator, Works) {
  const Runfiles* files = Runfiles::CreateForTest();
  const std::string images_directory =
      files->Rlocation("_main/testdata/calibration");

  EXPECT_THAT(
      Calibrate(images_directory),
      IsOkAndHolds(Partially(Approximately(EqualsProto(R"pb(camera_matrix: {
                                                              fx: 1419.3
                                                              fy: 1424.7
                                                              cx: 574.2
                                                              cy: 953.4
                                                            })pb"),
                                           kTolerance))));
}

}  // namespace
}  // namespace sfm