#ifndef RGBD_CAMERAS_FACTORY_HPP_
#define RGBD_CAMERAS_FACTORY_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "rgbd/core/camera.hpp"

namespace rgbd {

struct CameraUri {
  std::string scheme;
  std::string source;
  std::map<std::string, std::string> parameters;
};

struct CameraStreamConfig {
  std::size_t width = 848;
  std::size_t height = 530;
  int fps = 30;
  std::string format = "y16";
};

struct CameraDepthControlConfig {
  bool auto_exposure = true;
  int exposure_us = 15600;
  int gain = 16;
  int disparity_search_range = 128;
  int ae_max_exposure_us = 30158;
};

struct CameraConfig {
  std::string backend = "gemini305";
  std::string id = "gemini305";
  std::string serial;
  CameraStreamConfig depth{848, 530, 30, "y16"};
  CameraStreamConfig color{848, 530, 30, "rgb8"};
  CameraDepthControlConfig depth_control;
  bool align_depth_to_color = true;
  std::string source;
  bool loop = false;
  float depth_m = 2.0F;
};

std::optional<CameraUri> ParseCameraUri(std::string_view uri);
std::unique_ptr<IDepthCamera> CreateCamera(const CameraConfig& config);
std::unique_ptr<IDepthCamera> CreateCamera(const CameraUri& uri);
std::unique_ptr<IDepthCamera> CreateCameraFromUri(std::string_view uri);
std::vector<std::string> AvailableCameraSchemes();

}

#endif
