#ifndef APPS_APP_SUPPORT_HPP_
#define APPS_APP_SUPPORT_HPP_

#include <filesystem>
#include <stdexcept>
#include <string>

#include "rgbd/cameras/factory.hpp"
#include "rgbd/core/config.hpp"

namespace rgbd {
namespace app {

constexpr char kDefaultConfigPath[] = "config/rgbd.yaml";

struct ConfigArgument {
  std::string path = kDefaultConfigPath;
  bool explicit_path = false;
};

inline ConfigArgument ParseConfigArgument(int argc, char** argv) {
  ConfigArgument argument;
  for (int index = 1; index < argc; ++index) {
    const std::string value(argv[index]);
    if (value == "--config") {
      if (index + 1 >= argc) {
        throw std::invalid_argument("--config requires a path");
      }
      argument.path = argv[++index];
      argument.explicit_path = true;
    }
  }
  return argument;
}

inline void LoadConfig(Config& config, const ConfigArgument& argument) {
  if (!argument.explicit_path && !std::filesystem::exists(argument.path)) {
    return;
  }
  config.Load(argument.path);
}

inline int ParsePositiveInt(const std::string& value, const char* name) {
  try {
    std::size_t parsed = 0;
    const int result = std::stoi(value, &parsed);
    if (parsed != value.size() || result <= 0) {
      throw std::invalid_argument(std::string(name) + " must be positive");
    }
    return result;
  } catch (const std::invalid_argument&) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  } catch (const std::out_of_range&) {
    throw std::invalid_argument(std::string(name) + " is out of range");
  }
}

struct CameraOptions {
  CameraConfig camera;
  int timeout_ms = 1000;
};

inline int LoadPositiveInt(const Config& config, const std::string& path,
                           int fallback) {
  const int value = config.GetInt(path, fallback);
  if (value <= 0) {
    throw std::invalid_argument(path + " must be positive");
  }
  return value;
}

inline CameraOptions LoadCameraOptions(const Config& config) {
  CameraOptions options;
  CameraConfig& camera = options.camera;
  camera.backend = config.GetString("camera.backend", camera.backend);
  camera.id = config.GetString("camera.id", camera.id);
  camera.serial = config.GetString("camera.serial", camera.serial);
  camera.depth.width = static_cast<std::size_t>(LoadPositiveInt(
      config, "camera.depth.width", static_cast<int>(camera.depth.width)));
  camera.depth.height = static_cast<std::size_t>(LoadPositiveInt(
      config, "camera.depth.height", static_cast<int>(camera.depth.height)));
  camera.depth.fps =
      LoadPositiveInt(config, "camera.depth.fps", camera.depth.fps);
  camera.depth.format =
      config.GetString("camera.depth.format", camera.depth.format);
  camera.depth_control.auto_exposure = config.GetBool(
      "camera.depth.auto_exposure", camera.depth_control.auto_exposure);
  camera.depth_control.exposure_us = LoadPositiveInt(
      config, "camera.depth.exposure_us", camera.depth_control.exposure_us);
  camera.depth_control.gain =
      LoadPositiveInt(config, "camera.depth.gain", camera.depth_control.gain);
  camera.depth_control.disparity_search_range =
      LoadPositiveInt(config, "camera.depth.disparity_search_range",
                      camera.depth_control.disparity_search_range);
  camera.depth_control.ae_max_exposure_us =
      LoadPositiveInt(config, "camera.depth.ae_max_exposure_us",
                      camera.depth_control.ae_max_exposure_us);
  camera.color.width = static_cast<std::size_t>(LoadPositiveInt(
      config, "camera.color.width", static_cast<int>(camera.color.width)));
  camera.color.height = static_cast<std::size_t>(LoadPositiveInt(
      config, "camera.color.height", static_cast<int>(camera.color.height)));
  camera.color.fps =
      LoadPositiveInt(config, "camera.color.fps", camera.color.fps);
  camera.color.format =
      config.GetString("camera.color.format", camera.color.format);
  camera.align_depth_to_color = config.GetBool("camera.align_depth_to_color",
                                               camera.align_depth_to_color);
  camera.source = config.GetString("camera.source", camera.source);
  camera.loop = config.GetBool("camera.loop", camera.loop);
  camera.depth_m = config.GetFloat("camera.depth_m", camera.depth_m);
  options.timeout_ms =
      LoadPositiveInt(config, "camera.timeout_ms", options.timeout_ms);
  if (camera.backend.empty()) {
    throw std::invalid_argument("camera.backend must not be empty");
  }
  if (camera.id.empty()) {
    throw std::invalid_argument("camera.id must not be empty");
  }
  return options;
}

inline std::string DescribeCamera(const CameraConfig& camera) {
  return camera.backend + " depth=" + std::to_string(camera.depth.width) + "x" +
         std::to_string(camera.depth.height) + "@" +
         std::to_string(camera.depth.fps) + "fps/" + camera.depth.format +
         " color=" + std::to_string(camera.color.width) + "x" +
         std::to_string(camera.color.height) + "@" +
         std::to_string(camera.color.fps) + "fps/" + camera.color.format +
         " align=" + (camera.align_depth_to_color ? "true" : "false") +
         " exposure=" +
         (camera.depth_control.auto_exposure
              ? "auto"
              : std::to_string(camera.depth_control.exposure_us) + "us") +
         " gain=" + std::to_string(camera.depth_control.gain) + " disparity=" +
         std::to_string(camera.depth_control.disparity_search_range);
}

}  // namespace app
}  // namespace rgbd

#endif
