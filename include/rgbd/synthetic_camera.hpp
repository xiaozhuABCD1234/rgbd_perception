#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rgbd/camera.hpp"

namespace rgbd {

struct SyntheticCameraConfig {
    std::string camera_id = "synthetic";
    std::size_t width = 640;
    std::size_t height = 480;
    float depth_m = 2.0F;
    std::chrono::milliseconds frame_period{33};
};

/// Deterministic SDK-free source used by the demo and tests; it does not perform object recognition.
class SyntheticCamera final : public IDepthCamera {
public:
    explicit SyntheticCamera(SyntheticCameraConfig config = {});
    ~SyntheticCamera() override;

    void open() override;
    void start() override;
    void stop() override;
    void close() override;
    bool read(FrameSet& frame, std::chrono::milliseconds timeout) override;

    [[nodiscard]] std::string name() const override;

private:
    SyntheticCameraConfig config_;
    bool opened_ = false;
    bool streaming_ = false;
    std::uint64_t next_sequence_ = 0;
};

}  // namespace rgbd
