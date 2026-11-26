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

std::optional<CameraUri> ParseCameraUri(std::string_view uri);
std::unique_ptr<IDepthCamera> CreateCamera(const CameraUri& uri);
std::unique_ptr<IDepthCamera> CreateCameraFromUri(std::string_view uri);
std::vector<std::string> AvailableCameraSchemes();

}

#endif
