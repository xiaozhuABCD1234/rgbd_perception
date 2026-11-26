#ifndef RGBD_CORE_PERCEPTION_PIPELINE_HPP_
#define RGBD_CORE_PERCEPTION_PIPELINE_HPP_

#include <vector>

#include "rgbd/core/depth_estimator.hpp"
#include "rgbd/core/frame.hpp"
#include "rgbd/core/object_geometry.hpp"
#include "rgbd/core/observation.hpp"
#include "rgbd/core/recognizer.hpp"

namespace rgbd {

class PerceptionPipeline {
 public:
  PerceptionPipeline(
      IVisionRecognizer& recognizer, const IDepthEstimator& depth_estimator,
      const IObjectGeometryEstimator* geometry_estimator = nullptr) noexcept;

  [[nodiscard]] std::vector<ObjectObservation> Process(
      const FrameSet& frame) const;

 private:
  IVisionRecognizer* recognizer_ = nullptr;
  const IDepthEstimator* depth_estimator_ = nullptr;
  const IObjectGeometryEstimator* geometry_estimator_ = nullptr;
};

}

#endif
