#ifndef RGBD_INFERENCE_ENGINE_HPP_
#define RGBD_INFERENCE_ENGINE_HPP_

#include <string>

#include "rgbd/inference/tensor.hpp"

namespace rgbd {

class IInferenceEngine {
 public:
  virtual ~IInferenceEngine() = default;

  virtual void Load(const std::string& model_path) = 0;
  [[nodiscard]] virtual TensorMap Infer(const TensorMap& inputs) = 0;
  [[nodiscard]] virtual std::string Name() const = 0;
};

}

#endif
