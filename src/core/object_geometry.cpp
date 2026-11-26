#include "rgbd/core/object_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    const auto x = static_cast<std::uint32_t>(key.x);
    const auto y = static_cast<std::uint32_t>(key.y);
    const auto z = static_cast<std::uint32_t>(key.z);
    return (static_cast<std::size_t>(x) * 73856093U) ^
           (static_cast<std::size_t>(y) * 19349663U) ^
           (static_cast<std::size_t>(z) * 83492791U);
  }
};

struct VoxelAccumulator {
  Vector3 sum;
  std::size_t count = 0;
};

using VoxelMap = std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash>;

bool Finite(const Point3f& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

VoxelKey KeyFor(const Point3f& point, float voxel_size) {
  return {static_cast<int>(std::floor(point.x / voxel_size)),
          static_cast<int>(std::floor(point.y / voxel_size)),
          static_cast<int>(std::floor(point.z / voxel_size))};
}

double Dot(const Vector3& left, const Vector3& right) noexcept {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept {
  return {left.y * right.z - left.z * right.y,
          left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

Vector3 Normalize(const Vector3& value) noexcept {
  const double length = std::sqrt(Dot(value, value));
  if (length <= 1e-12) {
    return {};
  }
  return {value.x / length, value.y / length, value.z / length};
}

void JacobiEigen(const std::array<double, 9>& input,
                 std::array<double, 9>* eigenvectors,
                 std::array<double, 3>* eigenvalues) {
  std::array<double, 9> matrix = input;
  std::array<double, 9> vectors{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (int iteration = 0; iteration < 32; ++iteration) {
    int p = 0;
    int q = 1;
    double largest = std::fabs(matrix[1]);
    for (const auto& pair :
         {std::pair<int, int>{0, 2}, std::pair<int, int>{1, 2}}) {
      const double value = std::fabs(matrix[pair.first * 3 + pair.second]);
      if (value > largest) {
        largest = value;
        p = pair.first;
        q = pair.second;
      }
    }
    if (largest < 1e-12) {
      break;
    }
    const double app = matrix[p * 3 + p];
    const double aqq = matrix[q * 3 + q];
    const double apq = matrix[p * 3 + q];
    const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);

    std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    rotation[p * 3 + p] = cosine;
    rotation[q * 3 + q] = cosine;
    rotation[p * 3 + q] = sine;
    rotation[q * 3 + p] = -sine;

    std::array<double, 9> next{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        for (int inner = 0; inner < 3; ++inner) {
          next[row * 3 + column] +=
              rotation[inner * 3 + row] * matrix[inner * 3 + column];
        }
      }
    }
    matrix = {};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        for (int inner = 0; inner < 3; ++inner) {
          matrix[row * 3 + column] +=
              next[row * 3 + inner] * rotation[inner * 3 + column];
        }
      }
    }
    std::array<double, 9> next_vectors{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        for (int inner = 0; inner < 3; ++inner) {
          next_vectors[row * 3 + column] +=
              vectors[row * 3 + inner] * rotation[inner * 3 + column];
        }
      }
    }
    vectors = next_vectors;
  }

  *eigenvectors = vectors;
  *eigenvalues = {matrix[0], matrix[4], matrix[8]};
}

std::array<float, 9> RotationFromAxes(const Vector3& x_axis,
                                      const Vector3& y_axis,
                                      const Vector3& z_axis) {
  return {static_cast<float>(x_axis.x), static_cast<float>(y_axis.x),
          static_cast<float>(z_axis.x), static_cast<float>(x_axis.y),
          static_cast<float>(y_axis.y), static_cast<float>(z_axis.y),
          static_cast<float>(x_axis.z), static_cast<float>(y_axis.z),
          static_cast<float>(z_axis.z)};
}

OrientedBox FitBox(const std::vector<Point3f>& points, float min_size_m) {
  Vector3 mean;
  for (const Point3f& point : points) {
    mean.x += point.x;
    mean.y += point.y;
    mean.z += point.z;
  }
  const double inverse_count = 1.0 / static_cast<double>(points.size());
  mean.x *= inverse_count;
  mean.y *= inverse_count;
  mean.z *= inverse_count;

  std::array<double, 9> covariance{};
  for (const Point3f& point : points) {
    const Vector3 centered{point.x - mean.x, point.y - mean.y,
                           point.z - mean.z};
    covariance[0] += centered.x * centered.x;
    covariance[1] += centered.x * centered.y;
    covariance[2] += centered.x * centered.z;
    covariance[4] += centered.y * centered.y;
    covariance[5] += centered.y * centered.z;
    covariance[8] += centered.z * centered.z;
  }
  covariance[3] = covariance[1];
  covariance[6] = covariance[2];
  covariance[7] = covariance[5];
  for (double& value : covariance) {
    value *= inverse_count;
  }

  std::array<double, 9> eigenvectors{};
  std::array<double, 3> eigenvalues{};
  JacobiEigen(covariance, &eigenvectors, &eigenvalues);
  std::array<int, 3> sorted = {0, 1, 2};
  std::sort(sorted.begin(), sorted.end(), [&eigenvalues](int left, int right) {
    return eigenvalues[left] > eigenvalues[right];
  });

  Vector3 x_axis{eigenvectors[sorted[0]], eigenvectors[3 + sorted[0]],
                 eigenvectors[6 + sorted[0]]};
  Vector3 y_axis{eigenvectors[sorted[1]], eigenvectors[3 + sorted[1]],
                 eigenvectors[6 + sorted[1]]};
  x_axis = Normalize(x_axis);
  y_axis = Normalize(y_axis);
  if (Dot(x_axis, x_axis) <= 0.5 || Dot(y_axis, y_axis) <= 0.5) {
    x_axis = {1.0, 0.0, 0.0};
    y_axis = {0.0, 1.0, 0.0};
  }
  Vector3 z_axis = Normalize(Cross(x_axis, y_axis));
  y_axis = Normalize(Cross(z_axis, x_axis));
  x_axis = Normalize(Cross(y_axis, z_axis));

  const std::array<Vector3, 3> axes{x_axis, y_axis, z_axis};
  std::array<double, 3> minimum{std::numeric_limits<double>::max(),
                                std::numeric_limits<double>::max(),
                                std::numeric_limits<double>::max()};
  std::array<double, 3> maximum{std::numeric_limits<double>::lowest(),
                                std::numeric_limits<double>::lowest(),
                                std::numeric_limits<double>::lowest()};
  for (const Point3f& point : points) {
    const Vector3 relative{point.x, point.y, point.z};
    for (std::size_t axis = 0; axis < axes.size(); ++axis) {
      const double projection = Dot(relative, axes[axis]);
      minimum[axis] = std::min(minimum[axis], projection);
      maximum[axis] = std::max(maximum[axis], projection);
    }
  }

  OrientedBox box;
  const double center_x = (minimum[0] + maximum[0]) / 2.0;
  const double center_y = (minimum[1] + maximum[1]) / 2.0;
  const double center_z = (minimum[2] + maximum[2]) / 2.0;
  box.center = {static_cast<float>(x_axis.x * center_x + y_axis.x * center_y +
                                   z_axis.x * center_z),
                static_cast<float>(x_axis.y * center_x + y_axis.y * center_y +
                                   z_axis.y * center_z),
                static_cast<float>(x_axis.z * center_x + y_axis.z * center_y +
                                   z_axis.z * center_z)};
  box.size = {
      std::max(static_cast<float>(maximum[0] - minimum[0]), min_size_m),
      std::max(static_cast<float>(maximum[1] - minimum[1]), min_size_m),
      std::max(static_cast<float>(maximum[2] - minimum[2]), min_size_m)};
  box.rotation = RotationFromAxes(x_axis, y_axis, z_axis);
  return box;
}

std::optional<ObjectGeometry> SelectLargestCluster(
    const VoxelMap& voxels, std::size_t min_cluster_points,
    float min_box_size) {
  std::unordered_map<VoxelKey, bool, VoxelKeyHash> visited;
  visited.reserve(voxels.size());
  std::vector<Point3f> best_points;
  std::size_t best_count = 0;

  for (const auto& entry : voxels) {
    if (visited[entry.first]) {
      continue;
    }
    std::vector<VoxelKey> queue{entry.first};
    visited[entry.first] = true;
    std::vector<Point3f> cluster_points;
    std::size_t count = 0;
    for (std::size_t index = 0; index < queue.size(); ++index) {
      const VoxelKey key = queue[index];
      const auto found = voxels.find(key);
      if (found == voxels.end()) {
        continue;
      }
      const VoxelAccumulator& accumulator = found->second;
      cluster_points.push_back(
          {static_cast<float>(accumulator.sum.x /
                              static_cast<double>(accumulator.count)),
           static_cast<float>(accumulator.sum.y /
                              static_cast<double>(accumulator.count)),
           static_cast<float>(accumulator.sum.z /
                              static_cast<double>(accumulator.count))});
      count += accumulator.count;
      for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
          for (int z = -1; z <= 1; ++z) {
            if (x == 0 && y == 0 && z == 0) {
              continue;
            }
            const VoxelKey neighbor{key.x + x, key.y + y, key.z + z};
            if (voxels.find(neighbor) != voxels.end() && !visited[neighbor]) {
              visited[neighbor] = true;
              queue.push_back(neighbor);
            }
          }
        }
      }
    }
    if (count > best_count) {
      best_count = count;
      best_points = std::move(cluster_points);
    }
  }

  if (best_count < min_cluster_points || best_points.empty()) {
    return std::nullopt;
  }
  ObjectGeometry geometry;
  geometry.point_count = best_count;
  geometry.box = FitBox(best_points, min_box_size);
  return geometry;
}

}

DepthPointCloudGeometryEstimator::DepthPointCloudGeometryEstimator(
    ObjectGeometryConfig config)
    : config_(config) {
  if (config_.pixel_stride == 0) {
    throw std::invalid_argument("pixel_stride must be greater than zero");
  }
  if (config_.max_samples == 0) {
    throw std::invalid_argument("max_samples must be greater than zero");
  }
  if (!std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0.0F) {
    throw std::invalid_argument("voxel_size_m must be greater than zero");
  }
  if (config_.min_cluster_points == 0) {
    throw std::invalid_argument("min_cluster_points must be greater than zero");
  }
  if (!std::isfinite(config_.min_depth_m) ||
      !std::isfinite(config_.max_depth_m) || config_.min_depth_m < 0.0F ||
      config_.max_depth_m <= config_.min_depth_m) {
    throw std::invalid_argument("invalid depth range");
  }
  if (!std::isfinite(config_.min_box_size_m) ||
      config_.min_box_size_m <= 0.0F) {
    throw std::invalid_argument("min_box_size_m must be greater than zero");
  }
}

std::optional<ObjectGeometry> DepthPointCloudGeometryEstimator::Estimate(
    const FrameSet& frame, const Detection2D& detection) const {
  if (!frame.Valid() || !detection.box.Valid() ||
      !frame.color_intrinsics.Valid()) {
    return std::nullopt;
  }

  const int left = std::max(0, detection.box.x);
  const int top = std::max(0, detection.box.y);
  const int right = std::min(static_cast<int>(frame.depth.width),
                             detection.box.x + detection.box.width);
  const int bottom = std::min(static_cast<int>(frame.depth.height),
                              detection.box.y + detection.box.height);
  if (right <= left || bottom <= top) {
    return std::nullopt;
  }

  const bool use_mask = detection.mask.has_value() && detection.mask->Valid() &&
                        detection.mask->width == frame.depth.width &&
                        detection.mask->height == frame.depth.height;
  const std::size_t mask_stride_width =
      use_mask ? detection.mask->StrideWidth() : 0;
  const std::size_t area = static_cast<std::size_t>(right - left) *
                           static_cast<std::size_t>(bottom - top);
  std::size_t stride = config_.pixel_stride;
  while (stride < static_cast<std::size_t>(right - left) &&
         area / (stride * stride) > config_.max_samples) {
    ++stride;
  }
  const std::size_t expected_samples =
      std::min(config_.max_samples, area / (stride * stride) + 1);
  VoxelMap voxels;
  voxels.reserve(expected_samples);
  for (int y = top; y < bottom; y += static_cast<int>(stride)) {
    for (int x = left; x < right; x += static_cast<int>(stride)) {
      if (use_mask) {
        const std::size_t mask_index =
            static_cast<std::size_t>(y / detection.mask->stride) *
                mask_stride_width +
            static_cast<std::size_t>(x / detection.mask->stride);
        if (detection.mask->values[mask_index] == 0) {
          continue;
        }
      }
      const std::size_t depth_index =
          static_cast<std::size_t>(y) * frame.depth.width +
          static_cast<std::size_t>(x);
      const float depth = frame.depth.values[depth_index];
      if (!std::isfinite(depth) || depth < config_.min_depth_m ||
          depth > config_.max_depth_m) {
        continue;
      }
      const std::optional<Point3f> point =
          Unproject(frame.color_intrinsics, static_cast<float>(x),
                    static_cast<float>(y), depth);
      if (!point.has_value() || !Finite(*point)) {
        continue;
      }
      VoxelAccumulator& accumulator =
          voxels[KeyFor(*point, config_.voxel_size_m)];
      accumulator.sum.x += point->x;
      accumulator.sum.y += point->y;
      accumulator.sum.z += point->z;
      ++accumulator.count;
    }
  }

  return SelectLargestCluster(voxels, config_.min_cluster_points,
                              config_.min_box_size_m);
}

}
