#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "rgbd/detector.hpp"
#include "rgbd/frame.hpp"

namespace rgbd {

enum class DepthQuality {
  valid,
  invalid_frame,
  invalid_roi,
  invalid_intrinsics,
  too_few_valid_pixels,
};

inline constexpr const char* to_string(DepthQuality quality) noexcept {
  switch (quality) {
    case DepthQuality::valid:
      return "valid";
    case DepthQuality::invalid_frame:
      return "invalid_frame";
    case DepthQuality::invalid_roi:
      return "invalid_roi";
    case DepthQuality::invalid_intrinsics:
      return "invalid_intrinsics";
    case DepthQuality::too_few_valid_pixels:
      return "too_few_valid_pixels";
  }
  return "unknown";
}

struct Point3f {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct DepthEstimate {
  DepthQuality quality = DepthQuality::invalid_frame;
  std::size_t valid_pixel_count = 0;
  std::optional<float> distance_m;
  std::optional<Point3f> position_camera_m;
};

struct ObjectObservation {
  std::string camera_id;
  std::uint64_t sequence = 0;
  Nanoseconds timestamp{0};
  Detection2D detection;
  DepthEstimate depth;
};

}  // namespace rgbd
