#ifndef RGBD_CAMERAS_SYNTHETIC_CAMERA_HPP_
#define RGBD_CAMERAS_SYNTHETIC_CAMERA_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rgbd/core/camera.hpp"

namespace rgbd {

struct SyntheticCameraConfig {
  std::string camera_id = "synthetic";
  std::size_t width = 640;
  std::size_t height = 480;
  float depth_m = 2.0F;
  std::chrono::milliseconds frame_period{33};
};

class SyntheticCamera final : public IDepthCamera {
 public:
  explicit SyntheticCamera(SyntheticCameraConfig config = {});
  ~SyntheticCamera() override;

  void Open() override;
  void Start() override;
  void Stop() override;
  void Close() override;
  bool Read(FrameSet& frame, std::chrono::milliseconds timeout) override;

  [[nodiscard]] std::string Name() const override;

 private:
  SyntheticCameraConfig config_;
  bool opened_ = false;
  bool streaming_ = false;
  std::uint64_t next_sequence_ = 0;
};

}

#endif
