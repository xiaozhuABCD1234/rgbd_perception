#include "rgbd/core/depth_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace rgbd {

namespace {

constexpr float kMadNormalization = 1.4826F;

std::int64_t ClampIndex(std::int64_t value, std::int64_t low,
                        std::int64_t high) {
  return std::max(low, std::min(value, high));
}

}

RobustDepthFusionEstimator::RobustDepthFusionEstimator(DepthFusionConfig config)
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
  if (config_.mad_scale < 0.0F) {
    throw std::invalid_argument("mad_scale must not be negative");
  }
}

DepthEstimate RobustDepthFusionEstimator::Estimate(
    const FrameSet& frame, const Detection2D& detection) const {
  if (!frame.Valid()) {
    return {DepthQuality::kInvalidFrame, {}, 0, std::nullopt, std::nullopt};
  }
  if (!detection.box.Valid()) {
    return {DepthQuality::kInvalidRoi, {}, 0, std::nullopt, std::nullopt};
  }
  if (!frame.color_intrinsics.Valid()) {
    return {
        DepthQuality::kInvalidIntrinsics, {}, 0, std::nullopt, std::nullopt};
  }

  const auto width = static_cast<std::int64_t>(frame.depth.width);
  const auto height = static_cast<std::int64_t>(frame.depth.height);
  const double center_x = static_cast<double>(detection.box.x) +
                          static_cast<double>(detection.box.width) / 2.0;
  const double center_y = static_cast<double>(detection.box.y) +
                          static_cast<double>(detection.box.height) / 2.0;
  const double roi_width =
      static_cast<double>(detection.box.width) * config_.roi_scale;
  const double roi_height =
      static_cast<double>(detection.box.height) * config_.roi_scale;

  const std::int64_t left = ClampIndex(
      static_cast<std::int64_t>(std::floor(center_x - roi_width / 2.0)), 0,
      width);
  const std::int64_t right = ClampIndex(
      static_cast<std::int64_t>(std::ceil(center_x + roi_width / 2.0)), 0,
      width);
  const std::int64_t top = ClampIndex(
      static_cast<std::int64_t>(std::floor(center_y - roi_height / 2.0)), 0,
      height);
  const std::int64_t bottom = ClampIndex(
      static_cast<std::int64_t>(std::ceil(center_y + roi_height / 2.0)), 0,
      height);

  Rect roi;
  roi.x = static_cast<int>(left);
  roi.y = static_cast<int>(top);
  roi.width = static_cast<int>(right - left);
  roi.height = static_cast<int>(bottom - top);

  if (!roi.Valid()) {
    return {DepthQuality::kInvalidRoi, roi, 0, std::nullopt, std::nullopt};
  }

  std::vector<float> valid_depths;
  valid_depths.reserve(static_cast<std::size_t>(roi.width) *
                       static_cast<std::size_t>(roi.height));
  for (std::int64_t y = top; y < bottom; ++y) {
    for (std::int64_t x = left; x < right; ++x) {
      const auto index = static_cast<std::size_t>(y) * frame.depth.width +
                         static_cast<std::size_t>(x);
      const float depth = frame.depth.values[index];
      if (std::isfinite(depth) && depth >= config_.min_depth_m &&
          depth <= config_.max_depth_m) {
        valid_depths.push_back(depth);
      }
    }
  }

  if (valid_depths.size() < config_.min_valid_pixels) {
    return {DepthQuality::kTooFewValidPixels, roi, valid_depths.size(),
            std::nullopt, std::nullopt};
  }

  std::sort(valid_depths.begin(), valid_depths.end());
  const std::size_t middle = valid_depths.size() / 2;
  const float median =
      valid_depths.size() % 2 == 0
          ? (valid_depths[middle - 1] + valid_depths[middle]) / 2.0F
          : valid_depths[middle];

  std::vector<float> deviations;
  deviations.reserve(valid_depths.size());
  for (const float depth : valid_depths) {
    deviations.push_back(std::fabs(depth - median));
  }
  std::sort(deviations.begin(), deviations.end());
  const float mad = deviations[deviations.size() / 2];

  float fused_depth = median;
  std::size_t inlier_count = valid_depths.size();
  if (config_.mad_scale > 0.0F && mad > 0.0F) {
    const float threshold = config_.mad_scale * kMadNormalization * mad;
    float sum = 0.0F;
    inlier_count = 0;
    for (const float depth : valid_depths) {
      if (std::fabs(depth - median) <= threshold) {
        sum += depth;
        ++inlier_count;
      }
    }
    if (inlier_count >= config_.min_valid_pixels) {
      fused_depth = sum / static_cast<float>(inlier_count);
    } else {
      inlier_count = valid_depths.size();
    }
  }

  std::optional<Point3f> position;
  const std::int64_t center_pixel_x =
      ClampIndex(static_cast<std::int64_t>(std::floor(center_x)), 0, width - 1);
  const std::int64_t center_pixel_y = ClampIndex(
      static_cast<std::int64_t>(std::floor(center_y)), 0, height - 1);
  if (valid_depths.size() > 0) {
    position =
        Unproject(frame.color_intrinsics, static_cast<float>(center_pixel_x),
                  static_cast<float>(center_pixel_y), fused_depth);
  }

  return {DepthQuality::kValid, roi, inlier_count, fused_depth, position};
}

}
