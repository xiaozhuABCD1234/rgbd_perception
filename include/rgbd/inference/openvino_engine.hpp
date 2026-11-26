#ifndef RGBD_INFERENCE_OPENVINO_ENGINE_HPP_
#define RGBD_INFERENCE_OPENVINO_ENGINE_HPP_

#include <memory>
#include <string>

#include "rgbd/inference/engine.hpp"

namespace rgbd {

struct OpenVinoConfig {
  std::string device = "CPU";
};

class OpenVinoEngine final : public IInferenceEngine {
 public:
  explicit OpenVinoEngine(OpenVinoConfig config = {});
  ~OpenVinoEngine() override;

  OpenVinoEngine(const OpenVinoEngine&) = delete;
  OpenVinoEngine& operator=(const OpenVinoEngine&) = delete;

  void Load(const std::string& model_path) override;
  [[nodiscard]] TensorMap Infer(const TensorMap& inputs) override;
  [[nodiscard]] std::string Name() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rgbd

#endif
