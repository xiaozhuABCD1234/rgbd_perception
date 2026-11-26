#ifndef RGBD_CORE_GEOMETRY_HPP_
#define RGBD_CORE_GEOMETRY_HPP_

#include <array>
#include <cmath>
#include <optional>

#include "rgbd/core/frame.hpp"

namespace rgbd {

struct Point3f {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct RigidTransform {
  std::array<float, 9> rotation{
      {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F}};
  Point3f translation;

  static RigidTransform Identity() noexcept { return {}; }

  [[nodiscard]] Point3f Apply(const Point3f& point) const noexcept {
    return {
        rotation[0] * point.x + rotation[1] * point.y + rotation[2] * point.z +
            translation.x,
        rotation[3] * point.x + rotation[4] * point.y + rotation[5] * point.z +
            translation.y,
        rotation[6] * point.x + rotation[7] * point.y + rotation[8] * point.z +
            translation.z,
    };
  }
};

inline std::optional<Point3f> Unproject(const CameraIntrinsics& intrinsics,
                                        float pixel_x, float pixel_y,
                                        float depth_m) noexcept {
  if (!intrinsics.Valid() || !std::isfinite(depth_m) || depth_m <= 0.0F) {
    return std::nullopt;
  }
  return Point3f{(pixel_x - intrinsics.cx) / intrinsics.fx * depth_m,
                 (pixel_y - intrinsics.cy) / intrinsics.fy * depth_m, depth_m};
}

}

#endif
