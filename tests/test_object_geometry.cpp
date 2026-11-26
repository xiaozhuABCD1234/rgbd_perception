#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "rgbd/core/object_geometry.hpp"
#include "test_support.hpp"

namespace {

rgbd::FrameSet MakeFrame(std::size_t width, std::size_t height, float depth) {
  rgbd::FrameSet frame;
  frame.camera_id = "geometry";
  frame.sequence = 3;
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

void SetDepth(rgbd::FrameSet* frame, int left, int top, int width, int height,
              float depth) {
  for (int y = top; y < top + height; ++y) {
    for (int x = left; x < left + width; ++x) {
      frame->depth.values[static_cast<std::size_t>(y) * frame->depth.width +
                          static_cast<std::size_t>(x)] = depth;
    }
  }
}

rgbd::ObjectGeometryConfig Config() {
  rgbd::ObjectGeometryConfig config;
  config.pixel_stride = 1;
  config.voxel_size_m = 0.02F;
  config.min_cluster_points = 20;
  config.min_depth_m = 0.1F;
  config.max_depth_m = 10.0F;
  return config;
}

bool Orthonormal(const std::array<float, 9>& rotation) {
  const auto dot = [&rotation](int left, int right) {
    float value = 0.0F;
    for (int row = 0; row < 3; ++row) {
      value += rotation[row * 3 + left] * rotation[row * 3 + right];
    }
    return value;
  };
  return std::fabs(dot(0, 0) - 1.0F) < 1e-4F &&
         std::fabs(dot(1, 1) - 1.0F) < 1e-4F &&
         std::fabs(dot(2, 2) - 1.0F) < 1e-4F && std::fabs(dot(0, 1)) < 1e-4F &&
         std::fabs(dot(0, 2)) < 1e-4F && std::fabs(dot(1, 2)) < 1e-4F;
}

}

RGBD_TEST(object_geometry_selects_foreground_cluster) {
  rgbd::FrameSet frame = MakeFrame(64, 48, 3.0F);
  SetDepth(&frame, 20, 16, 24, 16, 1.0F);
  const rgbd::Detection2D detection{
      "object", 0.9F, {16, 12, 32, 24}, std::nullopt};
  const rgbd::DepthPointCloudGeometryEstimator estimator(Config());
  const std::optional<rgbd::ObjectGeometry> geometry =
      estimator.Estimate(frame, detection);
  RGBD_CHECK(geometry.has_value());
  RGBD_CHECK(geometry->point_count >= 300);
  RGBD_CHECK_NEAR(geometry->box.center.z, 1.0F, 1e-3F);
  RGBD_CHECK(geometry->box.size.x > 0.3F);
  RGBD_CHECK(geometry->box.size.y > 0.2F);
  RGBD_CHECK(geometry->box.size.z < 0.02F);
  RGBD_CHECK(Orthonormal(geometry->box.rotation));
}

RGBD_TEST(object_geometry_uses_mask) {
  rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  SetDepth(&frame, 25, 20, 10, 10, 1.0F);
  rgbd::Mask2D mask;
  mask.width = frame.depth.width;
  mask.height = frame.depth.height;
  mask.values.assign(mask.width * mask.height, 0);
  for (int y = 20; y < 30; ++y) {
    for (int x = 25; x < 35; ++x) {
      mask.values[static_cast<std::size_t>(y) * mask.width +
                  static_cast<std::size_t>(x)] = 1;
    }
  }
  const rgbd::Detection2D detection{"object", 0.9F, {0, 0, 64, 48}, mask};
  const rgbd::DepthPointCloudGeometryEstimator estimator(Config());
  const std::optional<rgbd::ObjectGeometry> geometry =
      estimator.Estimate(frame, detection);
  RGBD_CHECK(geometry.has_value());
  RGBD_CHECK(geometry->point_count == 100);
  RGBD_CHECK_NEAR(geometry->box.center.z, 1.0F, 1e-3F);
}

RGBD_TEST(object_geometry_rejects_too_few_points) {
  rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  SetDepth(&frame, 20, 16, 2, 2, 1.0F);
  rgbd::ObjectGeometryConfig config = Config();
  config.min_cluster_points = 20;
  const rgbd::DepthPointCloudGeometryEstimator estimator(config);
  const std::optional<rgbd::ObjectGeometry> geometry = estimator.Estimate(
      frame, {"object", 0.9F, {16, 12, 32, 24}, std::nullopt});
  RGBD_CHECK(!geometry.has_value());
}

RGBD_TEST(object_geometry_rejects_invalid_inputs) {
  const rgbd::DepthPointCloudGeometryEstimator estimator(Config());
  RGBD_CHECK(!estimator
                  .Estimate(rgbd::FrameSet{},
                            {"object", 0.9F, {0, 0, 1, 1}, std::nullopt})
                  .has_value());
  rgbd::FrameSet frame = MakeFrame(64, 48, 2.0F);
  RGBD_CHECK(
      !estimator.Estimate(frame, {"object", 0.9F, {0, 0, 0, 0}, std::nullopt})
           .has_value());
}

RGBD_TEST(object_geometry_validates_config) {
  RGBD_CHECK_THROW(rgbd::DepthPointCloudGeometryEstimator(
                       {0, 0.02F, 20, 0.1F, 10.0F, 0.01F}),
                   std::invalid_argument);
  RGBD_CHECK_THROW(
      rgbd::DepthPointCloudGeometryEstimator({1, 0.0F, 20, 0.1F, 10.0F, 0.01F}),
      std::invalid_argument);
  RGBD_CHECK_THROW(
      rgbd::DepthPointCloudGeometryEstimator({1, 0.02F, 0, 0.1F, 10.0F, 0.01F}),
      std::invalid_argument);
  RGBD_CHECK_THROW(rgbd::DepthPointCloudGeometryEstimator(
                       {1, 0.02F, 20, 10.0F, 1.0F, 0.01F}),
                   std::invalid_argument);
  RGBD_CHECK_THROW(
      rgbd::DepthPointCloudGeometryEstimator({1, 0.02F, 20, 0.1F, 10.0F, 0.0F}),
      std::invalid_argument);
}
