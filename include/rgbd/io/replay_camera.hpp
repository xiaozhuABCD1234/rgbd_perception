#ifndef RGBD_IO_REPLAY_CAMERA_HPP_
#define RGBD_IO_REPLAY_CAMERA_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rgbd/core/camera.hpp"

namespace rgbd {

struct ReplayCameraConfig {
  std::string path;
  bool loop = false;
  std::string camera_id = "replay";
};

class ReplayCamera final : public IDepthCamera {
 public:
  explicit ReplayCamera(ReplayCameraConfig config);
  ~ReplayCamera() override;

  ReplayCamera(const ReplayCamera&) = delete;
  ReplayCamera& operator=(const ReplayCamera&) = delete;

  void Open() override;
  void Start() override;
  void Stop() override;
  void Close() override;
  bool Read(FrameSet& frame, std::chrono::milliseconds timeout) override;

  [[nodiscard]] std::string Name() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}

#endif
