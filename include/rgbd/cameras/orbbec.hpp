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
  std::size_t depth_width = 848;
  std::size_t depth_height = 530;
  int depth_fps = 30;
  std::size_t color_width = 848;
  std::size_t color_height = 530;
  int color_fps = 30;
  bool align_depth_to_color = true;
};

std::unique_ptr<IDepthCamera> CreateOrbbecCamera(
    OrbbecCameraConfig config = {});

}

#endif
