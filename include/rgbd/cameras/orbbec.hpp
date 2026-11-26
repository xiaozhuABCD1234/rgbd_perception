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
  bool depth_auto_exposure = true;
  int depth_exposure_us = 15600;
  int depth_gain = 16;
  int disparity_search_range = 128;
  int ae_max_exposure_us = 30158;
};

std::unique_ptr<IDepthCamera> CreateOrbbecCamera(
    OrbbecCameraConfig config = {});

}

#endif
