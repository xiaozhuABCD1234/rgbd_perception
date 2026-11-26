#include "rgbd/inference/openvino_engine.hpp"

#include <openvino/openvino.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

ov::Shape ToShape(const std::vector<std::int64_t>& shape) {
  ov::Shape result;
  result.reserve(shape.size());
  for (const std::int64_t dimension : shape) {
    if (dimension < 0) {
      throw std::invalid_argument("OpenVINO input shape is not static");
    }
    result.push_back(static_cast<std::size_t>(dimension));
  }
  return result;
}

}  // namespace

class OpenVinoEngine::Impl {
 public:
  explicit Impl(OpenVinoConfig engine_config)
      : config(std::move(engine_config)) {
    if (config.device.empty()) {
      throw std::invalid_argument("OpenVINO device must not be empty");
    }
  }

  void Load(const std::string& model_path) {
    if (model_path.empty()) {
      throw std::invalid_argument("OpenVINO model path must not be empty");
    }
    core = std::make_unique<ov::Core>();
    const auto model = core->read_model(model_path);
    compiled = std::make_unique<ov::CompiledModel>(
        core->compile_model(model, config.device));
  }

  [[nodiscard]] TensorMap Infer(const TensorMap& inputs) {
    if (compiled == nullptr) {
      throw std::logic_error("OpenVINO model is not loaded");
    }
    ov::InferRequest request = compiled->create_infer_request();
    for (const auto& port : compiled->inputs()) {
      const auto found = inputs.find(port.get_any_name());
      const Tensor* tensor = nullptr;
      if (found != inputs.end()) {
        tensor = &found->second;
      } else if (inputs.size() == 1) {
        tensor = &inputs.begin()->second;
      } else {
        throw std::invalid_argument("missing OpenVINO input: " +
                                    port.get_any_name());
      }
      if (tensor->DataType() != TensorDataType::kFloat32) {
        throw std::invalid_argument("OpenVINO input must be float32");
      }
      const ov::Tensor input_tensor(ov::element::f32, ToShape(tensor->Shape()),
                                    tensor->Bytes().data());
      request.set_tensor(port, input_tensor);
    }
    request.infer();

    TensorMap outputs;
    std::size_t index = 0;
    for (const auto& port : compiled->outputs()) {
      const ov::Tensor output = request.get_tensor(port);
      if (output.get_element_type() != ov::element::f32) {
        throw std::runtime_error("OpenVINO output is not float32");
      }
      const auto shape = output.get_shape();
      std::vector<std::int64_t> tensor_shape;
      tensor_shape.reserve(shape.size());
      for (const std::size_t dimension : shape) {
        tensor_shape.push_back(static_cast<std::int64_t>(dimension));
      }
      const std::size_t count = output.get_size();
      const float* data = output.data<float>();
      std::vector<float> values(data, data + count);
      std::string name = port.get_any_name();
      if (name.empty()) {
        name = "output_" + std::to_string(index);
      }
      outputs.emplace(
          name, Tensor::Float32(std::move(tensor_shape), std::move(values)));
      ++index;
    }
    return outputs;
  }

  [[nodiscard]] std::string Name() const { return "openvino:" + config.device; }

  OpenVinoConfig config;
  std::unique_ptr<ov::Core> core;
  std::unique_ptr<ov::CompiledModel> compiled;
};

OpenVinoEngine::OpenVinoEngine(OpenVinoConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

OpenVinoEngine::~OpenVinoEngine() = default;

void OpenVinoEngine::Load(const std::string& model_path) {
  impl_->Load(model_path);
}

TensorMap OpenVinoEngine::Infer(const TensorMap& inputs) {
  return impl_->Infer(inputs);
}

std::string OpenVinoEngine::Name() const { return impl_->Name(); }

}  // namespace rgbd
