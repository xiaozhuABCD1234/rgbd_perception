#include "rgbd/cameras/synthetic_camera.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace rgbd {

SyntheticCamera::SyntheticCamera(SyntheticCameraConfig config)
    : config_(std::move(config)) {
  if (config_.camera_id.empty() || config_.width == 0 || config_.height == 0) {
    throw std::invalid_argument(
        "synthetic camera requires an ID and non-zero dimensions");
  }
  if (config_.depth_m <= 0.0F) {
    throw std::invalid_argument("synthetic depth must be positive");
  }
}

SyntheticCamera::~SyntheticCamera() { Close(); }

void SyntheticCamera::Open() {
  if (opened_) {
    return;
  }
  opened_ = true;
  next_sequence_ = 0;
}

void SyntheticCamera::Start() {
  if (!opened_) {
    throw std::logic_error("camera must be opened before start");
  }
  streaming_ = true;
}

void SyntheticCamera::Stop() { streaming_ = false; }

void SyntheticCamera::Close() {
  Stop();
  opened_ = false;
}

bool SyntheticCamera::Read(FrameSet& frame, std::chrono::milliseconds timeout) {
  if (!streaming_) {
    return false;
  }

  if (config_.frame_period.count() > 0 && timeout.count() > 0) {
    std::this_thread::sleep_for(std::min(timeout, config_.frame_period));
  }

  frame = FrameSet{};
  frame.camera_id = config_.camera_id;
  frame.sequence = next_sequence_;
  frame.device_timestamp =
      std::chrono::duration_cast<Nanoseconds>(config_.frame_period) *
      static_cast<std::int64_t>(next_sequence_);
  frame.host_timestamp = std::chrono::duration_cast<Nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch());

  frame.color.width = config_.width;
  frame.color.height = config_.height;
  frame.color.pixels.resize(config_.width * config_.height *
                            ColorImage::channels);
  for (std::size_t y = 0; y < config_.height; ++y) {
    for (std::size_t x = 0; x < config_.width; ++x) {
      const auto offset = (y * config_.width + x) * ColorImage::channels;
      frame.color.pixels[offset] = static_cast<std::uint8_t>(x % 256);
      frame.color.pixels[offset + 1] = static_cast<std::uint8_t>(y % 256);
      frame.color.pixels[offset + 2] =
          static_cast<std::uint8_t>(next_sequence_ % 256);
    }
  }

  frame.depth.width = config_.width;
  frame.depth.height = config_.height;
  frame.depth.values.assign(config_.width * config_.height, config_.depth_m);
  frame.color_intrinsics = {
      config_.width,
      config_.height,
      static_cast<float>(config_.width),
      static_cast<float>(config_.width),
      static_cast<float>(config_.width - 1) / 2.0F,
      static_cast<float>(config_.height - 1) / 2.0F,
  };

  ++next_sequence_;
  return true;
}

std::string SyntheticCamera::Name() const { return config_.camera_id; }

}
