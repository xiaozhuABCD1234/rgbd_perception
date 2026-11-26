#include <stdexcept>
#include <vector>

#include "rgbd/inference/tensor.hpp"
#include "test_support.hpp"

namespace {

using rgbd::Tensor;
using rgbd::TensorDataType;

RGBD_TEST(tensor_float32_roundtrip) {
  const Tensor tensor =
      Tensor::Float32({2, 3}, {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
  RGBD_CHECK(tensor.DataType() == TensorDataType::kFloat32);
  RGBD_CHECK(tensor.Shape().size() == 2);
  RGBD_CHECK(tensor.ElementCount() == 6);
  const std::vector<float> values = tensor.AsFloat32();
  RGBD_CHECK(values.size() == 6);
  RGBD_CHECK_NEAR(values[5], 6.0F, 1e-6F);
}

RGBD_TEST(tensor_uint8_roundtrip) {
  const Tensor tensor = Tensor::UInt8({1, 4}, {1, 2, 3, 4});
  RGBD_CHECK(tensor.DataType() == TensorDataType::kUInt8);
  RGBD_CHECK(tensor.ElementCount() == 4);
  RGBD_CHECK(tensor.Bytes().size() == 4);
  RGBD_CHECK(tensor.Bytes()[3] == 4);
  RGBD_CHECK_THROW(static_cast<void>(tensor.AsFloat32()),
                   std::invalid_argument);
}

RGBD_TEST(tensor_rejects_mismatched_shape) {
  RGBD_CHECK_THROW(Tensor::Float32({2, 2}, {1.0F, 2.0F}),
                   std::invalid_argument);
  RGBD_CHECK_THROW(Tensor::Float32({-1}, {}), std::invalid_argument);
  RGBD_CHECK_THROW(Tensor::UInt8({2}, {1}), std::invalid_argument);
}

}
