#include <fstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "calibrator.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/text_format.h"
#include "status_macros.h"

ABSL_FLAG(std::string, images_directory, "testdata/calibration",
          "Directory for chessboard images");
ABSL_FLAG(std::string, output_calibration_proto,
          "/tmp/pixel_6a_calibration.txtpb", "Intrinsic camera calibration");

absl::Status Run() {
  ASSIGN_OR_RETURN(auto calibration,
                   sfm::Calibrate(absl::GetFlag(FLAGS_images_directory)));
  std::string text_format;
  google::protobuf::TextFormat::PrintToString(calibration, &text_format);
  std::ofstream output_file(absl::GetFlag(FLAGS_output_calibration_proto));
  if (!output_file) {
    return absl::InternalError(absl::StrCat(
        "Failed writing to ", absl::GetFlag(FLAGS_output_calibration_proto)));
  }
  output_file << text_format;
  output_file.close();
  LOG(INFO) << "Calibration complete: " << text_format;
  return absl::OkStatus();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");
  if (const auto status = Run(); !status.ok()) {
    LOG(ERROR) << "Failed to calibrate: " << status.message();
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}