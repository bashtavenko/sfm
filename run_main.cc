#include <fstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/text_format.h"
#include "reconstruction.h"
#include "sfm.h"
#include "status_macros.h"

ABSL_FLAG(std::string, images_directory, "testdata/reconstruction",
          "Directory with input image frames");
ABSL_FLAG(std::string, calibration_file, "testdata/pixel_6a_calibration.txtpb",
          "Intrinsic camera calibration");
ABSL_FLAG(std::string, ouput_point_cloud_file, "/tmp/bottle.ply",
          "Point cloud in PLY format");

absl::Status Run() {
  ASSIGN_OR_RETURN(auto camera_matrix,
                   sfm::LoadCameraMatrixFromTextProtoFile(
                       absl::GetFlag(FLAGS_calibration_file)));
  sfm::Reconstruction reconstruction(camera_matrix);
  ASSIGN_OR_RETURN(
      auto image_paths,
      sfm::GetFilesFromDirectory(absl::GetFlag(FLAGS_images_directory)));

  RETURN_IF_ERROR(reconstruction.Run(
      image_paths, absl::GetFlag(FLAGS_ouput_point_cloud_file)));

  return absl::OkStatus();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");
  if (const auto status = Run(); !status.ok()) {
    LOG(ERROR) << "Failed: " << status.message();
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
