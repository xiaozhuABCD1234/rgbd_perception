#ifndef RGBD_CORE_OBJECT_GEOMETRY_HPP_
#define RGBD_CORE_OBJECT_GEOMETRY_HPP_

#include <array>
#include <cstddef>
#include <optional>

#include "rgbd/core/frame.hpp"
#include "rgbd/core/geometry.hpp"
#include "rgbd/core/recognizer.hpp"

namespace rgbd {

struct OrientedBox {
  Point3f center;
  Point3f size;
  std::array<float, 9> rotation{
      {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F}};
};

struct ObjectGeometry {
  std::size_t point_count = 0;
  OrientedBox box;
};

struct ObjectGeometryConfig {
  std::size_t pixel_stride = 2;
  float voxel_size_m = 0.025F;
  std::size_t min_cluster_points = 20;
  float min_depth_m = 0.1F;
  float max_depth_m = 10.0F;
  float min_box_size_m = 0.01F;
  std::size_t max_samples = 2000;
};

class IObjectGeometryEstimator {
 public:
  virtual ~IObjectGeometryEstimator() = default;

  [[nodiscard]] virtual std::optional<ObjectGeometry> Estimate(
      const FrameSet& frame, const Detection2D& detection) const = 0;
};

class DepthPointCloudGeometryEstimator final : public IObjectGeometryEstimator {
 public:
  explicit DepthPointCloudGeometryEstimator(ObjectGeometryConfig config = {});

  [[nodiscard]] std::optional<ObjectGeometry> Estimate(
      const FrameSet& frame, const Detection2D& detection) const override;

 private:
  ObjectGeometryConfig config_;
};

}

#endif
