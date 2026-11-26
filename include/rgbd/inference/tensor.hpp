#ifndef RGBD_INFERENCE_TENSOR_HPP_
#define RGBD_INFERENCE_TENSOR_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rgbd {

enum class TensorDataType {
  kFloat32,
  kUInt8,
};

class Tensor {
 public:
  Tensor() = default;

  static Tensor Float32(std::vector<std::int64_t> shape,
                        std::vector<float> values);
  static Tensor UInt8(std::vector<std::int64_t> shape,
                      std::vector<std::uint8_t> values);

  [[nodiscard]] TensorDataType DataType() const noexcept { return data_type_; }
  [[nodiscard]] const std::vector<std::int64_t>& Shape() const noexcept {
    return shape_;
  }
  [[nodiscard]] std::int64_t ElementCount() const noexcept;
  [[nodiscard]] const std::vector<std::uint8_t>& Bytes() const noexcept {
    return bytes_;
  }
  [[nodiscard]] std::vector<float> AsFloat32() const;

 private:
  Tensor(TensorDataType data_type, std::vector<std::int64_t> shape,
         std::vector<std::uint8_t> bytes);

  TensorDataType data_type_ = TensorDataType::kFloat32;
  std::vector<std::int64_t> shape_;
  std::vector<std::uint8_t> bytes_;
};

using TensorMap = std::map<std::string, Tensor>;

}

#endif
