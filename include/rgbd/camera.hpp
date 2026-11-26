#pragma once

#include <chrono>
#include <string>

#include "rgbd/frame.hpp"

namespace rgbd {

class IDepthCamera {
 public:
  virtual ~IDepthCamera() = default;

  virtual void open() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void close() = 0;

  virtual bool read(FrameSet& frame, std::chrono::milliseconds timeout) = 0;

  [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace rgbd
