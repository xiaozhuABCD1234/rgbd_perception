#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rgbd/recognition/yolo26_recognizer.hpp"
#include "test_support.hpp"

namespace {

class FakeEngine final : public rgbd::IInferenceEngine {
 public:
  explicit FakeEngine(rgbd::TensorMap outputs) : outputs_(std::move(outputs)) {}

  void Load(const std::string&) override {}

  [[nodiscard]] rgbd::TensorMap Infer(const rgbd::TensorMap&) override {
    return outputs_;
  }

  [[nodiscard]] std::string Name() const override { return "fake"; }

 private:
  rgbd::TensorMap outputs_;
};

rgbd::FrameSet MakeFrame() {
  rgbd::FrameSet frame;
  frame.camera_id = "yolo";
  frame.color.width = 100;
  frame.color.height = 100;
  frame.color.pixels.assign(100 * 100 * rgbd::ColorImage::channels, 0);
  return frame;
}

rgbd::TensorMap DetectionOutputs() {
  std::vector<float> values(5 * 10, 0.0F);
  const auto set = [&](int anchor, int channel, float value) {
    values[static_cast<std::size_t>(channel) * 10 + anchor] = value;
  };
  set(0, 0, 320.0F);
  set(0, 1, 320.0F);
  set(0, 2, 100.0F);
  set(0, 3, 100.0F);
  set(0, 4, 0.9F);
  set(1, 0, 321.0F);
  set(1, 1, 321.0F);
  set(1, 2, 100.0F);
  set(1, 3, 100.0F);
  set(1, 4, 0.8F);
  return {{"output0", rgbd::Tensor::Float32({1, 5, 10}, std::move(values))}};
}

}

RGBD_TEST(yolo26_decodes_and_suppresses_detections) {
  rgbd::Yolo26Config config;
  config.labels = {"object"};
  rgbd::Yolo26Recognizer recognizer(
      std::make_shared<FakeEngine>(DetectionOutputs()), config);
  const auto detections = recognizer.Recognize(MakeFrame());
  RGBD_CHECK(detections.size() == 1);
  RGBD_CHECK(detections[0].label == "object");
  RGBD_CHECK_NEAR(detections[0].confidence, 0.9F, 1e-6F);
  RGBD_CHECK(detections[0].box.Valid());
}

RGBD_TEST(yolo26_decodes_end_to_end_output) {
  std::vector<float> values(300 * 6, 0.0F);
  values[0] = 100.0F;
  values[1] = 100.0F;
  values[2] = 200.0F;
  values[3] = 200.0F;
  values[4] = 0.75F;
  values[5] = 0.0F;
  rgbd::Yolo26Config config;
  config.labels = {"object"};
  rgbd::Yolo26Recognizer recognizer(
      std::make_shared<FakeEngine>(rgbd::TensorMap{
          {"output0", rgbd::Tensor::Float32({1, 300, 6}, std::move(values))}}),
      config);
  const auto detections = recognizer.Recognize(MakeFrame());
  RGBD_CHECK(detections.size() == 1);
  RGBD_CHECK_NEAR(detections[0].confidence, 0.75F, 1e-6F);
}

RGBD_TEST(yolo26_decodes_segmentation_mask) {
  std::vector<float> detection_values(6 * 10, 0.0F);
  const auto set = [&](int anchor, int channel, float value) {
    detection_values[static_cast<std::size_t>(channel) * 10 + anchor] = value;
  };
  set(0, 0, 320.0F);
  set(0, 1, 320.0F);
  set(0, 2, 100.0F);
  set(0, 3, 100.0F);
  set(0, 4, 0.9F);
  set(0, 5, 10.0F);
  std::vector<float> prototype_values(1 * 1 * 2 * 2, 10.0F);
  rgbd::Yolo26Config config;
  config.labels = {"object"};
  rgbd::Yolo26Recognizer recognizer(
      std::make_shared<FakeEngine>(rgbd::TensorMap{
          {"output0",
           rgbd::Tensor::Float32({1, 6, 10}, std::move(detection_values))},
          {"output1",
           rgbd::Tensor::Float32({1, 1, 2, 2}, std::move(prototype_values))}}),
      config);
  const auto detections = recognizer.Recognize(MakeFrame());
  RGBD_CHECK(detections.size() == 1);
  RGBD_CHECK(detections[0].mask.has_value());
  RGBD_CHECK(detections[0].mask->Valid());
  RGBD_CHECK(detections[0].mask->stride > 1);
  RGBD_CHECK(detections[0].mask->values[0] == 255);
}

RGBD_TEST(yolo26_rejects_invalid_config) {
  RGBD_CHECK_THROW(rgbd::Yolo26Recognizer(nullptr), std::invalid_argument);
  rgbd::Yolo26Config config;
  config.confidence_threshold = 2.0F;
  RGBD_CHECK_THROW(rgbd::Yolo26Recognizer(
                       std::make_shared<FakeEngine>(rgbd::TensorMap{}), config),
                   std::invalid_argument);
}
