#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/status/status.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_processing_toolbox.h"

ABSL_FLAG(std::string, input_video_path, "testdata/video_10s.mp4",
          "Input video path file.");
ABSL_FLAG(int32_t, number_frames, 10,
          "Number of frames to take from the input file.");
ABSL_FLAG(std::string, output_directory, "/tmp/frames",
          "Directory which have the frame files.");

absl::Status Run() { return sfm::SaveFrames(absl::GetFlag(FLAGS_input_video_path),
                         absl::GetFlag(FLAGS_number_frames),
                         absl::GetFlag(FLAGS_output_directory));
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");
  auto status = Run();
  if (!status.ok()) {
    LOG(ERROR) << status;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}