#pragma once

#include <chrono>
#include <string>

#include "rgbd/frame.hpp"

namespace rgbd {

/// Control-plane interface for a depth camera. Implementations must contain SDK types privately.
class IDepthCamera {
public:
    virtual ~IDepthCamera() = default;

    virtual void open() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void close() = 0;

    /// Returns false when no frame is available before the timeout or the camera is not streaming.
    virtual bool read(FrameSet& frame, std::chrono::milliseconds timeout) = 0;

    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace rgbd
