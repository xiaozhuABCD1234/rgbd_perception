#pragma once

#include <cstddef>

#include "rgbd/observation.hpp"

namespace rgbd {

class IDepthEstimator {
public:
    virtual ~IDepthEstimator() = default;

    [[nodiscard]] virtual DepthEstimate estimate(const FrameSet& frame,
                                                 const Detection2D& detection) const = 0;
};

struct CenterPatchDepthEstimatorConfig {
    float roi_scale = 0.5F;
    std::size_t min_valid_pixels = 5;
    float min_depth_m = 0.1F;
    float max_depth_m = 10.0F;
};

/// Robust baseline implementation: median depth in a central patch of the detection box.
class CenterPatchDepthEstimator final : public IDepthEstimator {
public:
    explicit CenterPatchDepthEstimator(CenterPatchDepthEstimatorConfig config = {});

    [[nodiscard]] DepthEstimate estimate(const FrameSet& frame,
                                         const Detection2D& detection) const override;

private:
    CenterPatchDepthEstimatorConfig config_;
};

}  // namespace rgbd
