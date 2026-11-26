#include "rgbd/inference/tensor.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

std::int64_t ShapeElementCount(const std::vector<std::int64_t>& shape) {
  if (shape.empty()) {
    return 0;
  }
  std::int64_t count = 1;
  for (const std::int64_t dimension : shape) {
    if (dimension < 0) {
      throw std::invalid_argument("tensor shape contains a negative dimension");
    }
    if (dimension != 0 &&
        count > std::numeric_limits<std::int64_t>::max() / dimension) {
      throw std::invalid_argument("tensor shape is too large");
    }
    count *= dimension;
  }
  return count;
}

}

Tensor::Tensor(TensorDataType data_type, std::vector<std::int64_t> shape,
               std::vector<std::uint8_t> bytes)
    : data_type_(data_type),
      shape_(std::move(shape)),
      bytes_(std::move(bytes)) {
  if (ElementCount() < 0) {
    throw std::invalid_argument("invalid tensor shape");
  }
}

Tensor Tensor::Float32(std::vector<std::int64_t> shape,
                       std::vector<float> values) {
  const std::int64_t count = ShapeElementCount(shape);
  if (count < 0 || static_cast<std::size_t>(count) != values.size()) {
    throw std::invalid_argument("tensor shape does not match its values");
  }
  std::vector<std::uint8_t> bytes(values.size() * sizeof(float));
  if (!values.empty()) {
    std::memcpy(bytes.data(), values.data(), bytes.size());
  }
  return Tensor(TensorDataType::kFloat32, std::move(shape), std::move(bytes));
}

Tensor Tensor::UInt8(std::vector<std::int64_t> shape,
                     std::vector<std::uint8_t> values) {
  const std::int64_t count = ShapeElementCount(shape);
  if (count < 0 || static_cast<std::size_t>(count) != values.size()) {
    throw std::invalid_argument("tensor shape does not match its values");
  }
  return Tensor(TensorDataType::kUInt8, std::move(shape), std::move(values));
}

std::int64_t Tensor::ElementCount() const noexcept {
  if (shape_.empty()) {
    return 0;
  }
  std::int64_t count = 1;
  for (const std::int64_t dimension : shape_) {
    if (dimension <= 0) {
      return 0;
    }
    count *= dimension;
  }
  return count;
}

std::vector<float> Tensor::AsFloat32() const {
  if (data_type_ != TensorDataType::kFloat32 ||
      bytes_.size() % sizeof(float) != 0) {
    throw std::invalid_argument("tensor is not float32");
  }
  std::vector<float> values(bytes_.size() / sizeof(float));
  if (!bytes_.empty()) {
    std::memcpy(values.data(), bytes_.data(), bytes_.size());
  }
  return values;
}

}
