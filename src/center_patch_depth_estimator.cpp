#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "rgbd/depth_estimator.hpp"

namespace rgbd {

CenterPatchDepthEstimator::CenterPatchDepthEstimator(
    CenterPatchDepthEstimatorConfig config)
    : config_(config) {
  if (config_.roi_scale <= 0.0F || config_.roi_scale > 1.0F) {
    throw std::invalid_argument("roi_scale must be in (0, 1]");
  }
  if (config_.min_valid_pixels == 0) {
    throw std::invalid_argument("min_valid_pixels must be greater than zero");
  }
  if (config_.min_depth_m < 0.0F ||
      config_.max_depth_m <= config_.min_depth_m) {
    throw std::invalid_argument("invalid depth range");
  }
}

DepthEstimate CenterPatchDepthEstimator::estimate(
    const FrameSet& frame, const Detection2D& detection) const {
  if (!frame.valid()) {
    return {DepthQuality::invalid_frame, 0, std::nullopt, std::nullopt};
  }
  if (detection.box.width <= 0 || detection.box.height <= 0) {
    return {DepthQuality::invalid_roi, 0, std::nullopt, std::nullopt};
  }

  const auto& intrinsics = frame.color_intrinsics;
  if (intrinsics.fx <= 0.0F || intrinsics.fy <= 0.0F) {
    return {DepthQuality::invalid_intrinsics, 0, std::nullopt, std::nullopt};
  }

  const auto image_width = static_cast<std::int64_t>(frame.depth.width);
  const auto image_height = static_cast<std::int64_t>(frame.depth.height);
  const double center_x = static_cast<double>(detection.box.x) +
                          static_cast<double>(detection.box.width) / 2.0;
  const double center_y = static_cast<double>(detection.box.y) +
                          static_cast<double>(detection.box.height) / 2.0;
  const double roi_width =
      static_cast<double>(detection.box.width) * config_.roi_scale;
  const double roi_height =
      static_cast<double>(detection.box.height) * config_.roi_scale;

  const auto left = std::max<std::int64_t>(
      0, static_cast<std::int64_t>(std::floor(center_x - roi_width / 2.0)));
  const auto right = std::min<std::int64_t>(
      image_width,
      static_cast<std::int64_t>(std::ceil(center_x + roi_width / 2.0)));
  const auto top = std::max<std::int64_t>(
      0, static_cast<std::int64_t>(std::floor(center_y - roi_height / 2.0)));
  const auto bottom = std::min<std::int64_t>(
      image_height,
      static_cast<std::int64_t>(std::ceil(center_y + roi_height / 2.0)));

  if (left >= right || top >= bottom) {
    return {DepthQuality::invalid_roi, 0, std::nullopt, std::nullopt};
  }

  std::vector<float> depths;
  depths.reserve(static_cast<std::size_t>((right - left) * (bottom - top)));
  for (std::int64_t y = top; y < bottom; ++y) {
    for (std::int64_t x = left; x < right; ++x) {
      const auto index = static_cast<std::size_t>(y) * frame.depth.width +
                         static_cast<std::size_t>(x);
      const float depth = frame.depth.values[index];
      if (std::isfinite(depth) && depth >= config_.min_depth_m &&
          depth <= config_.max_depth_m) {
        depths.push_back(depth);
      }
    }
  }

  if (depths.size() < config_.min_valid_pixels) {
    return {DepthQuality::too_few_valid_pixels, depths.size(), std::nullopt,
            std::nullopt};
  }

  std::sort(depths.begin(), depths.end());
  const std::size_t middle = depths.size() / 2;
  const float median = depths.size() % 2 == 0
                           ? (depths[middle - 1] + depths[middle]) / 2.0F
                           : depths[middle];

  const float normalized_x =
      static_cast<float>((center_x - intrinsics.cx) / intrinsics.fx);
  const float normalized_y =
      static_cast<float>((center_y - intrinsics.cy) / intrinsics.fy);
  const Point3f position{normalized_x * median, normalized_y * median, median};
  return {DepthQuality::valid, depths.size(), median, position};
}

}  // namespace rgbd
