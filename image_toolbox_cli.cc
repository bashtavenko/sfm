#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/status/status.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_processing_toolbox.h"
#include "status_macros.h"

ABSL_FLAG(std::string, input_video_path, "testdata/video.mp4",
          "Input video path file.");
ABSL_FLAG(std::string, output_directory, "/tmp/frames",
          "Directory which have the frame files.");

absl::Status Run() {
  RETURN_IF_ERROR(
      sfm::SaveRelatedFrames(absl::GetFlag(FLAGS_input_video_path), absl::
        GetFlag(FLAGS_output_directory)));
  return absl::OkStatus();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");
  if (auto status = Run(); !status.ok()) {
    LOG(ERROR) << status;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}