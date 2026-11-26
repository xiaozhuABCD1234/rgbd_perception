#ifndef RGBD_CORE_OBSERVATION_HPP_
#define RGBD_CORE_OBSERVATION_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "rgbd/core/frame.hpp"
#include "rgbd/core/geometry.hpp"
#include "rgbd/core/recognizer.hpp"

namespace rgbd {

enum class DepthQuality {
  kValid,
  kInvalidFrame,
  kInvalidRoi,
  kInvalidIntrinsics,
  kTooFewValidPixels,
};

inline const char* ToString(DepthQuality quality) noexcept {
  switch (quality) {
    case DepthQuality::kValid:
      return "valid";
    case DepthQuality::kInvalidFrame:
      return "invalid_frame";
    case DepthQuality::kInvalidRoi:
      return "invalid_roi";
    case DepthQuality::kInvalidIntrinsics:
      return "invalid_intrinsics";
    case DepthQuality::kTooFewValidPixels:
      return "too_few_valid_pixels";
  }
  return "unknown";
}

struct DepthEstimate {
  DepthQuality quality = DepthQuality::kInvalidFrame;
  Rect roi;
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

}

#endif
