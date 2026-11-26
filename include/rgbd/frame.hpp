#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rgbd {

using Nanoseconds = std::chrono::nanoseconds;

struct ColorImage {
  std::size_t width = 0;
  std::size_t height = 0;
  static constexpr std::size_t channels = 3;
  std::vector<std::uint8_t> pixels;

  [[nodiscard]] bool valid() const noexcept {
    return width > 0 && height > 0 &&
           pixels.size() == width * height * channels;
  }
};

struct DepthImage {
  std::size_t width = 0;
  std::size_t height = 0;
  std::vector<float> values;

  [[nodiscard]] bool valid() const noexcept {
    return width > 0 && height > 0 && values.size() == width * height;
  }
};

struct CameraIntrinsics {
  std::size_t width = 0;
  std::size_t height = 0;
  float fx = 0.0F;
  float fy = 0.0F;
  float cx = 0.0F;
  float cy = 0.0F;
};

struct FrameSet {
  std::string camera_id;
  std::uint64_t sequence = 0;
  Nanoseconds device_timestamp{0};
  Nanoseconds host_timestamp{0};
  ColorImage color;
  DepthImage depth;
  CameraIntrinsics color_intrinsics;

  [[nodiscard]] bool valid() const noexcept {
    return !camera_id.empty() && color.valid() && depth.valid() &&
           color.width == depth.width && color.height == depth.height &&
           color_intrinsics.width == color.width &&
           color_intrinsics.height == color.height;
  }
};

}  // namespace rgbd
