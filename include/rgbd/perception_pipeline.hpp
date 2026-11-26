#pragma once

#include <vector>

#include "rgbd/depth_estimator.hpp"
#include "rgbd/detector.hpp"
#include "rgbd/observation.hpp"

namespace rgbd {

/// Synchronous use-case pipeline. Thread scheduling and bounded queues belong to the application layer.
class PerceptionPipeline {
public:
    PerceptionPipeline(IObjectDetector& detector, const IDepthEstimator& depth_estimator) noexcept;

    [[nodiscard]] std::vector<ObjectObservation> process(const FrameSet& frame) const;

private:
    IObjectDetector& detector_;
    const IDepthEstimator& depth_estimator_;
};

}  // namespace rgbd
