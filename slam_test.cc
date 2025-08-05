#include "slam.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace sfm {
namespace {

TEST(Slam, InitWorks) {
  auto slam = SLAM();
}

} // namespace
} // nnamespace sfm