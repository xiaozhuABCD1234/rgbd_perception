#ifndef RGBD_CAMERAS_REALSENSE_HPP_
#define RGBD_CAMERAS_REALSENSE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "rgbd/core/camera.hpp"

namespace rgbd {

struct RealSenseCameraConfig {
  std::string camera_id = "d435i";
  std::string serial;
  std::size_t width = 640;
  std::size_t height = 480;
  int fps = 30;
};

std::unique_ptr<IDepthCamera> CreateRealSenseCamera(
    RealSenseCameraConfig config = {});

}

#endif
