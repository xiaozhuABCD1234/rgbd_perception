#ifndef RGBD_CORE_DEPTH_ESTIMATOR_HPP_
#define RGBD_CORE_DEPTH_ESTIMATOR_HPP_

#include <cstddef>

#include "rgbd/core/frame.hpp"
#include "rgbd/core/observation.hpp"
#include "rgbd/core/recognizer.hpp"

namespace rgbd {

struct DepthFusionConfig {
  float roi_scale = 0.75F;
  std::size_t min_valid_pixels = 5;
  float min_depth_m = 0.1F;
  float max_depth_m = 10.0F;
  float mad_scale = 3.0F;
};

class IDepthEstimator {
 public:
  virtual ~IDepthEstimator() = default;

  [[nodiscard]] virtual DepthEstimate Estimate(
      const FrameSet& frame, const Detection2D& detection) const = 0;
};

class RobustDepthFusionEstimator final : public IDepthEstimator {
 public:
  explicit RobustDepthFusionEstimator(DepthFusionConfig config = {});

  [[nodiscard]] DepthEstimate Estimate(
      const FrameSet& frame, const Detection2D& detection) const override;

 private:
  DepthFusionConfig config_;
};

}

#endif
