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
ABSL_FLAG(std::int32_t, threshold, 0,
          "Minimum feature count to output new image. Mutually exclusive with number_frames.");
ABSL_FLAG(int32_t, number_frames, 50,
          "Number of frames to take from the input file. Mutually exclusive with threshold.");

absl::Status Run() {
  if (absl::GetFlag(FLAGS_threshold) > 0 && !
      absl::GetFlag(FLAGS_number_frames)) {
    RETURN_IF_ERROR(
        sfm::SaveRelatedFrames(absl::GetFlag(FLAGS_input_video_path), absl::
          GetFlag(FLAGS_output_directory), absl::GetFlag(FLAGS_threshold)));
  } else if (!absl::GetFlag(FLAGS_threshold) && absl::GetFlag(
                 FLAGS_number_frames) > 0) {
    RETURN_IF_ERROR(
        sfm::SaveFrames(absl::GetFlag(FLAGS_input_video_path), absl::GetFlag(
            FLAGS_number_frames), absl::
          GetFlag(FLAGS_output_directory)));
  } else return absl::InvalidArgumentError("Mutually exclusive options");
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