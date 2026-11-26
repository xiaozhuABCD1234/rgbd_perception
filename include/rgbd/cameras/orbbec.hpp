#ifndef RGBD_CAMERAS_ORBBEC_HPP_
#define RGBD_CAMERAS_ORBBEC_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "rgbd/core/camera.hpp"

namespace rgbd {

struct OrbbecCameraConfig {
  std::string camera_id = "gemini305";
  std::string serial;
  std::size_t width = 640;
  std::size_t height = 480;
  int fps = 30;
};

std::unique_ptr<IDepthCamera> CreateOrbbecCamera(
    OrbbecCameraConfig config = {});

}

#endif
