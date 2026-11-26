#include "rgbd/cameras/factory.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rgbd/cameras/synthetic_camera.hpp"
#include "rgbd/io/replay_camera.hpp"

#ifdef RGBD_WITH_ORBBEC
#include "rgbd/cameras/orbbec.hpp"
#endif

#ifdef RGBD_WITH_REALSENSE
#include "rgbd/cameras/realsense.hpp"
#endif

namespace rgbd {

namespace {

bool IsSchemeCharacter(char value) {
  const auto character = static_cast<unsigned char>(value);
  return std::isalnum(character) != 0 || value == '+' || value == '-' ||
         value == '.';
}

std::string Lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

int HexValue(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  return -1;
}

std::optional<std::string> PercentDecode(std::string_view input) {
  std::string output;
  output.reserve(input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    if (input[index] != '%') {
      output.push_back(input[index]);
      continue;
    }
    if (index + 2 >= input.size()) {
      return std::nullopt;
    }
    const int high = HexValue(input[index + 1]);
    const int low = HexValue(input[index + 2]);
    if (high < 0 || low < 0) {
      return std::nullopt;
    }
    output.push_back(static_cast<char>((high << 4) | low));
    index += 2;
  }
  return output;
}

std::string StringOption(const CameraUri& uri, const std::string& key,
                         const std::string& fallback) {
  const auto found = uri.parameters.find(key);
  return found == uri.parameters.end() ? fallback : found->second;
}

int IntOption(const CameraUri& uri, const std::string& key, int fallback) {
  const auto found = uri.parameters.find(key);
  if (found == uri.parameters.end()) {
    return fallback;
  }
  std::size_t parsed = 0;
  const int value = std::stoi(found->second, &parsed);
  if (parsed != found->second.size()) {
    throw std::invalid_argument("invalid integer option: " + key);
  }
  return value;
}

float FloatOption(const CameraUri& uri, const std::string& key,
                  float fallback) {
  const auto found = uri.parameters.find(key);
  if (found == uri.parameters.end()) {
    return fallback;
  }
  std::size_t parsed = 0;
  const float value = std::stof(found->second, &parsed);
  if (parsed != found->second.size()) {
    throw std::invalid_argument("invalid float option: " + key);
  }
  return value;
}

bool BoolOption(const CameraUri& uri, const std::string& key, bool fallback) {
  const std::string value = Lowercase(StringOption(uri, key, ""));
  if (value.empty()) {
    return fallback;
  }
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::invalid_argument("invalid boolean option: " + key);
}

std::chrono::milliseconds FramePeriod(int fps) {
  if (fps <= 0) {
    throw std::invalid_argument("fps must be positive");
  }
  return std::chrono::milliseconds(1000 / fps);
}

}

std::optional<CameraUri> ParseCameraUri(std::string_view uri) {
  const std::size_t delimiter = uri.find("://");
  if (delimiter == std::string_view::npos || delimiter == 0) {
    return std::nullopt;
  }
  const std::string scheme = Lowercase(std::string(uri.substr(0, delimiter)));
  if (!std::all_of(scheme.begin(), scheme.end(), IsSchemeCharacter)) {
    return std::nullopt;
  }

  const std::string_view remainder = uri.substr(delimiter + 3);
  const std::size_t query_start = remainder.find('?');
  const std::string_view source_view = query_start == std::string_view::npos
                                           ? remainder
                                           : remainder.substr(0, query_start);
  const std::optional<std::string> source = PercentDecode(source_view);
  if (!source) {
    return std::nullopt;
  }

  CameraUri parsed;
  parsed.scheme = scheme;
  parsed.source = *source;

  if (query_start == std::string_view::npos) {
    return parsed;
  }

  const std::string_view query = remainder.substr(query_start + 1);
  std::size_t position = 0;
  while (position < query.size()) {
    const std::size_t segment_end = query.find('&', position);
    const std::string_view segment =
        query.substr(position, segment_end == std::string_view::npos
                                   ? std::string_view::npos
                                   : segment_end - position);
    if (!segment.empty()) {
      const std::size_t equals = segment.find('=');
      if (equals == std::string_view::npos || equals == 0) {
        return std::nullopt;
      }
      const std::optional<std::string> key =
          PercentDecode(segment.substr(0, equals));
      const std::optional<std::string> value =
          PercentDecode(segment.substr(equals + 1));
      if (!key || !value || key->empty()) {
        return std::nullopt;
      }
      parsed.parameters[*key] = *value;
    }
    if (segment_end == std::string_view::npos) {
      break;
    }
    position = segment_end + 1;
  }
  return parsed;
}

CameraConfig ConfigFromUri(const CameraUri& uri) {
  CameraConfig config;
  config.backend = uri.scheme;
  config.id = StringOption(uri, "id", uri.scheme);
  config.serial = StringOption(uri, "serial", "");
  const int width =
      IntOption(uri, "width", static_cast<int>(config.depth.width));
  const int height =
      IntOption(uri, "height", static_cast<int>(config.depth.height));
  const int fps = IntOption(uri, "fps", config.depth.fps);
  config.depth.width = static_cast<std::size_t>(width);
  config.depth.height = static_cast<std::size_t>(height);
  config.depth.fps = fps;
  config.color.width = static_cast<std::size_t>(width);
  config.color.height = static_cast<std::size_t>(height);
  config.color.fps = fps;
  config.depth.format = config.backend == "d435i" ? "z16" : "y16";
  config.color.format = "rgb8";
  config.align_depth_to_color =
      BoolOption(uri, "align_depth_to_color", config.align_depth_to_color);
  config.source = uri.source;
  config.loop = BoolOption(uri, "loop", config.loop);
  config.depth_m = FloatOption(uri, "depth_m", config.depth_m);
  return config;
}

std::unique_ptr<IDepthCamera> CreateCamera(const CameraUri& uri) {
  return CreateCamera(ConfigFromUri(uri));
}

std::unique_ptr<IDepthCamera> CreateCamera(const CameraConfig& config) {
  const std::string backend = Lowercase(config.backend);
  if (backend == "synthetic") {
    SyntheticCameraConfig synthetic;
    synthetic.camera_id = config.id;
    synthetic.width = config.color.width;
    synthetic.height = config.color.height;
    synthetic.depth_m = config.depth_m;
    synthetic.frame_period = FramePeriod(config.color.fps);
    return std::make_unique<SyntheticCamera>(std::move(synthetic));
  }
  if (backend == "replay") {
    if (config.source.empty()) {
      throw std::invalid_argument("replay source must not be empty");
    }
    ReplayCameraConfig replay;
    replay.path = config.source;
    replay.loop = config.loop;
    replay.camera_id = config.id;
    return std::make_unique<ReplayCamera>(std::move(replay));
  }
  if (backend == "gemini305") {
    if (Lowercase(config.depth.format) != "y16" ||
        Lowercase(config.color.format) != "rgb8") {
      throw std::invalid_argument(
          "gemini305 supports depth y16 and color rgb8");
    }
#ifdef RGBD_WITH_ORBBEC
    OrbbecCameraConfig orbbec;
    orbbec.camera_id = config.id;
    orbbec.serial = config.serial;
    orbbec.depth_width = config.depth.width;
    orbbec.depth_height = config.depth.height;
    orbbec.depth_fps = config.depth.fps;
    orbbec.color_width = config.color.width;
    orbbec.color_height = config.color.height;
    orbbec.color_fps = config.color.fps;
    orbbec.align_depth_to_color = config.align_depth_to_color;
    orbbec.depth_auto_exposure = config.depth_control.auto_exposure;
    orbbec.depth_exposure_us = config.depth_control.exposure_us;
    orbbec.depth_gain = config.depth_control.gain;
    orbbec.disparity_search_range = config.depth_control.disparity_search_range;
    orbbec.ae_max_exposure_us = config.depth_control.ae_max_exposure_us;
    return CreateOrbbecCamera(std::move(orbbec));
#else
    throw std::runtime_error("gemini305 backend is not built");
#endif
  }
  if (backend == "d435i") {
    if (Lowercase(config.depth.format) != "z16" ||
        Lowercase(config.color.format) != "rgb8") {
      throw std::invalid_argument("d435i supports depth z16 and color rgb8");
    }
#ifdef RGBD_WITH_REALSENSE
    RealSenseCameraConfig realsense;
    realsense.camera_id = config.id;
    realsense.serial = config.serial;
    realsense.depth_width = config.depth.width;
    realsense.depth_height = config.depth.height;
    realsense.depth_fps = config.depth.fps;
    realsense.color_width = config.color.width;
    realsense.color_height = config.color.height;
    realsense.color_fps = config.color.fps;
    realsense.align_depth_to_color = config.align_depth_to_color;
    return CreateRealSenseCamera(std::move(realsense));
#else
    throw std::runtime_error("d435i backend is not built");
#endif
  }
  throw std::invalid_argument("unknown camera backend: " + config.backend);
}

std::unique_ptr<IDepthCamera> CreateCameraFromUri(std::string_view uri) {
  const std::optional<CameraUri> parsed = ParseCameraUri(uri);
  if (!parsed) {
    throw std::invalid_argument("invalid camera URI");
  }
  return CreateCamera(*parsed);
}

std::vector<std::string> AvailableCameraSchemes() {
  std::vector<std::string> schemes{"synthetic", "replay"};
#ifdef RGBD_WITH_ORBBEC
  schemes.push_back("gemini305");
#endif
#ifdef RGBD_WITH_REALSENSE
  schemes.push_back("d435i");
#endif
  return schemes;
}
}
