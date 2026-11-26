#include <chrono>
#include <cstddef>
#include <stdexcept>

#include "rgbd/cameras/synthetic_camera.hpp"
#include "test_support.hpp"

namespace {

rgbd::SyntheticCameraConfig Config() {
  rgbd::SyntheticCameraConfig config;
  config.width = 16;
  config.height = 12;
  config.frame_period = std::chrono::milliseconds(0);
  return config;
}

RGBD_TEST(synthetic_camera_lifecycle) {
  rgbd::SyntheticCamera camera(Config());
  rgbd::FrameSet frame;
  RGBD_CHECK(!camera.Read(frame, std::chrono::milliseconds(0)));
  camera.Open();
  camera.Start();
  RGBD_CHECK(camera.Read(frame, std::chrono::milliseconds(1)));
  RGBD_CHECK(frame.Valid());
  RGBD_CHECK(frame.sequence == 0);
  RGBD_CHECK(camera.Read(frame, std::chrono::milliseconds(1)));
  RGBD_CHECK(frame.sequence == 1);
  camera.Stop();
  RGBD_CHECK(!camera.Read(frame, std::chrono::milliseconds(0)));
  camera.Close();
}

RGBD_TEST(synthetic_camera_validates_config) {
  rgbd::SyntheticCameraConfig config = Config();
  config.width = 0;
  RGBD_CHECK_THROW(rgbd::SyntheticCamera{config}, std::invalid_argument);
  config = Config();
  config.depth_m = 0.0F;
  RGBD_CHECK_THROW(rgbd::SyntheticCamera{config}, std::invalid_argument);
}

RGBD_TEST(synthetic_camera_requires_open_before_start) {
  rgbd::SyntheticCamera camera(Config());
  RGBD_CHECK_THROW(camera.Start(), std::logic_error);
}

}
