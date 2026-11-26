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

std::string SourceOrThrow(const CameraUri& uri) {
  if (uri.source.empty()) {
    throw std::invalid_argument("camera source must not be empty");
  }
  return uri.source;
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

std::unique_ptr<IDepthCamera> CreateCamera(const CameraUri& uri) {
  if (uri.scheme == "synthetic") {
    SyntheticCameraConfig config;
    config.camera_id = StringOption(uri, "id", config.camera_id);
    config.width = static_cast<std::size_t>(
        IntOption(uri, "width", static_cast<int>(config.width)));
    config.height = static_cast<std::size_t>(
        IntOption(uri, "height", static_cast<int>(config.height)));
    config.depth_m = FloatOption(uri, "depth_m", config.depth_m);
    config.frame_period = FramePeriod(IntOption(uri, "fps", 30));
    return std::make_unique<SyntheticCamera>(std::move(config));
  }
  if (uri.scheme == "replay") {
    ReplayCameraConfig config;
    config.path = SourceOrThrow(uri);
    config.loop = BoolOption(uri, "loop", false);
    config.camera_id = StringOption(uri, "id", config.camera_id);
    return std::make_unique<ReplayCamera>(std::move(config));
  }
  if (uri.scheme == "gemini305") {
#ifdef RGBD_WITH_ORBBEC
    OrbbecCameraConfig config;
    config.camera_id = StringOption(uri, "id", config.camera_id);
    config.serial = StringOption(uri, "serial", config.serial);
    config.width = static_cast<std::size_t>(
        IntOption(uri, "width", static_cast<int>(config.width)));
    config.height = static_cast<std::size_t>(
        IntOption(uri, "height", static_cast<int>(config.height)));
    config.fps = IntOption(uri, "fps", config.fps);
    return CreateOrbbecCamera(std::move(config));
#else
    throw std::runtime_error("gemini305 backend is not built");
#endif
  }
  if (uri.scheme == "d435i") {
#ifdef RGBD_WITH_REALSENSE
    RealSenseCameraConfig config;
    config.camera_id = StringOption(uri, "id", config.camera_id);
    config.serial = StringOption(uri, "serial", config.serial);
    config.width = static_cast<std::size_t>(
        IntOption(uri, "width", static_cast<int>(config.width)));
    config.height = static_cast<std::size_t>(
        IntOption(uri, "height", static_cast<int>(config.height)));
    config.fps = IntOption(uri, "fps", config.fps);
    return CreateRealSenseCamera(std::move(config));
#else
    throw std::runtime_error("d435i backend is not built");
#endif
  }
  throw std::invalid_argument("unknown camera scheme: " + uri.scheme);
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
