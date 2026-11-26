#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "rgbd/core/perception_pipeline.hpp"
#include "rgbd/recognition/center_box_recognizer.hpp"
#include "test_support.hpp"

namespace {

rgbd::FrameSet MakeFrame() {
  constexpr std::size_t width = 64;
  constexpr std::size_t height = 48;
  rgbd::FrameSet frame;
  frame.camera_id = "pipeline";
  frame.sequence = 42;
  frame.device_timestamp = std::chrono::nanoseconds(99);
  frame.color.width = width;
  frame.color.height = height;
  frame.color.pixels.assign(width * height * rgbd::ColorImage::channels, 0);
  frame.depth.width = width;
  frame.depth.height = height;
  frame.depth.values.assign(width * height, 1.5F);
  frame.color_intrinsics = {width, height, 50.0F, 50.0F, 31.5F, 23.5F};
  return frame;
}

class StaticRecognizer final : public rgbd::IVisionRecognizer {
 public:
  [[nodiscard]] std::vector<rgbd::Detection2D> Recognize(
      const rgbd::FrameSet&) override {
    return {{"box", 0.75F, {16, 12, 32, 24}, std::nullopt}};
  }
};

RGBD_TEST(pipeline_preserves_frame_identity) {
  const rgbd::FrameSet frame = MakeFrame();
  StaticRecognizer recognizer;
  const rgbd::RobustDepthFusionEstimator estimator;
  const rgbd::PerceptionPipeline pipeline(recognizer, estimator);
  const std::vector<rgbd::ObjectObservation> observations =
      pipeline.Process(frame);
  RGBD_CHECK(observations.size() == 1);
  RGBD_CHECK(observations.front().camera_id == frame.camera_id);
  RGBD_CHECK(observations.front().sequence == frame.sequence);
  RGBD_CHECK(observations.front().timestamp == frame.device_timestamp);
  RGBD_CHECK(observations.front().depth.quality == rgbd::DepthQuality::kValid);
}

RGBD_TEST(pipeline_returns_empty_without_detections) {
  const rgbd::FrameSet frame = MakeFrame();
  rgbd::CenterBoxRecognizer recognizer;
  const rgbd::RobustDepthFusionEstimator estimator;
  const rgbd::PerceptionPipeline pipeline(recognizer, estimator);
  const std::vector<rgbd::ObjectObservation> observations =
      pipeline.Process(frame);
  RGBD_CHECK(observations.size() == 1);
  RGBD_CHECK(observations.front().detection.label == "object");
  RGBD_CHECK(observations.front().depth.position_camera_m.has_value());
}

RGBD_TEST(center_box_recognizer_validates_config) {
  RGBD_CHECK_THROW(rgbd::CenterBoxRecognizer({"", 0.5F, 1.0F}),
                   std::invalid_argument);
  RGBD_CHECK_THROW(rgbd::CenterBoxRecognizer({"object", 1.5F, 1.0F}),
                   std::invalid_argument);
  RGBD_CHECK_THROW(rgbd::CenterBoxRecognizer({"object", 0.5F, 2.0F}),
                   std::invalid_argument);
}

}
