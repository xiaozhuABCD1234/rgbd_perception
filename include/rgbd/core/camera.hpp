#ifndef RGBD_CORE_CAMERA_HPP_
#define RGBD_CORE_CAMERA_HPP_

#include <chrono>
#include <string>

#include "rgbd/core/frame.hpp"

namespace rgbd {

class IDepthCamera {
 public:
  virtual ~IDepthCamera() = default;

  virtual void Open() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual void Close() = 0;

  virtual bool Read(FrameSet& frame, std::chrono::milliseconds timeout) = 0;

  [[nodiscard]] virtual std::string Name() const = 0;
};

}

#endif
