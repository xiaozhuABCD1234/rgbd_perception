#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "rgbd/core/depth_estimator.hpp"
#include "test_support.hpp"

namespace {

rgbd::FrameSet MakeFrame(std::size_t width, std::size_t height, float depth) {
  rgbd::FrameSet frame;
  frame.camera_id = "test";
  frame.sequence = 7;
  frame.device_timestamp = std::chrono::nanoseconds(123);
  frame.color.width = width;
  frame.color.height = height;
  frame.color.pixels.assign(width * height * rgbd::ColorImage::channels, 0);
  frame.depth.width = width;
  frame.depth.height = height;
  frame.depth.values.assign(width * height, depth);
  frame.color_intrinsics = {
      width,
      height,
      50.0F,
      50.0F,
      static_cast<float>(width - 1) / 2.0F,
      static_cast<float>(height - 1) / 2.0F,
  };
  return frame;
}

RGBD_TEST(depth_fusion_returns_center_position) {
  rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  frame.depth.values[0] = std::numeric_limits<float>::quiet_NaN();
  frame.depth.values[1] = 100.0F;
  const rgbd::RobustDepthFusionEstimator estimator;
  const rgbd::Detection2D detection{
      "object", 0.9F, {16, 12, 32, 24}, std::nullopt};
  const rgbd::DepthEstimate estimate = estimator.Estimate(frame, detection);
  RGBD_CHECK(estimate.quality == rgbd::DepthQuality::kValid);
  RGBD_CHECK(estimate.distance_m.has_value());
  RGBD_CHECK_NEAR(*estimate.distance_m, 2.0F, 1e-6F);
  RGBD_CHECK(estimate.position_camera_m.has_value());
  RGBD_CHECK_NEAR(estimate.position_camera_m->x, 0.02F, 1e-6F);
  RGBD_CHECK_NEAR(estimate.position_camera_m->y, 0.02F, 1e-6F);
  RGBD_CHECK_NEAR(estimate.position_camera_m->z, 2.0F, 1e-6F);
}

RGBD_TEST(depth_fusion_rejects_invalid_depth) {
  rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  frame.depth.values.assign(frame.depth.values.size(),
                            std::numeric_limits<float>::quiet_NaN());
  const rgbd::RobustDepthFusionEstimator estimator;
  const rgbd::DepthEstimate estimate = estimator.Estimate(
      frame, {"object", 0.9F, {16, 12, 32, 24}, std::nullopt});
  RGBD_CHECK(estimate.quality == rgbd::DepthQuality::kTooFewValidPixels);
  RGBD_CHECK(!estimate.distance_m.has_value());
  RGBD_CHECK(!estimate.position_camera_m.has_value());
}

RGBD_TEST(depth_fusion_clips_roi) {
  const rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  const rgbd::RobustDepthFusionEstimator estimator;
  const rgbd::DepthEstimate estimate =
      estimator.Estimate(frame, {"edge", 0.8F, {-8, -6, 16, 12}, std::nullopt});
  RGBD_CHECK(estimate.quality == rgbd::DepthQuality::kValid);
  RGBD_CHECK(estimate.roi.x == 0);
  RGBD_CHECK(estimate.roi.y == 0);
  RGBD_CHECK(estimate.roi.width > 0);
  RGBD_CHECK(estimate.roi.height > 0);
}

RGBD_TEST(depth_fusion_rejects_invalid_inputs) {
  const rgbd::RobustDepthFusionEstimator estimator;
  rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  RGBD_CHECK(estimator
                 .Estimate(rgbd::FrameSet{},
                           {"invalid", 0.0F, {0, 0, 1, 1}, std::nullopt})
                 .quality == rgbd::DepthQuality::kInvalidFrame);
  RGBD_CHECK(
      estimator.Estimate(frame, {"invalid", 0.0F, {0, 0, 0, 0}, std::nullopt})
          .quality == rgbd::DepthQuality::kInvalidRoi);
}

RGBD_TEST(depth_fusion_validates_config) {
  RGBD_CHECK_THROW(
      rgbd::RobustDepthFusionEstimator({0.0F, 5, 0.1F, 10.0F, 3.0F}),
      std::invalid_argument);
  RGBD_CHECK_THROW(
      rgbd::RobustDepthFusionEstimator({0.5F, 0, 0.1F, 10.0F, 3.0F}),
      std::invalid_argument);
  RGBD_CHECK_THROW(
      rgbd::RobustDepthFusionEstimator({0.5F, 5, 10.0F, 1.0F, 3.0F}),
      std::invalid_argument);
  RGBD_CHECK_THROW(
      rgbd::RobustDepthFusionEstimator({0.5F, 5, 0.1F, 10.0F, -1.0F}),
      std::invalid_argument);
}

}
