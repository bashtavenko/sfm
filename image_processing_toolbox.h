#ifndef IMAGE_PROCESSING_TOOLBOX_H
#define IMAGE_PROCESSING_TOOLBOX_H
#include <filesystem>
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace sfm {

// Saves k frames from the given video into output directory
absl::Status SaveFrames(absl::string_view video_path, int32_t k,
                        absl::string_view frame_output_directory);

}  // namespace sfm

#endif  // IMAGE_PROCESSING_TOOLBOX_H
