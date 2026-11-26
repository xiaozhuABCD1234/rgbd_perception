#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rgbd/recognition/detection_filter.hpp"
#include "test_support.hpp"

namespace {

class SequenceRecognizer final : public rgbd::IVisionRecognizer {
 public:
  explicit SequenceRecognizer(
      std::vector<std::vector<rgbd::Detection2D>> frames)
      : frames_(std::move(frames)) {}

  [[nodiscard]] std::vector<rgbd::Detection2D> Recognize(
      const rgbd::FrameSet&) override {
    if (next_ >= frames_.size()) {
      return {};
    }
    return frames_[next_++];
  }

 private:
  std::vector<std::vector<rgbd::Detection2D>> frames_;
  std::size_t next_ = 0;
};

rgbd::Detection2D Detection(int x, float confidence = 0.8F) {
  return {"cup", confidence, {x, 10, 20, 20}, std::nullopt};
}

}

RGBD_TEST(detection_filter_confirms_and_smooths_tracks) {
  std::vector<std::vector<rgbd::Detection2D>> frames{
      {Detection(10)}, {Detection(14)}, {}};
  rgbd::DetectionFilterConfig config;
  config.min_hits = 2;
  config.smoothing = 0.5F;
  config.min_iou = 0.1F;
  rgbd::FilteringRecognizer recognizer(
      std::make_unique<SequenceRecognizer>(std::move(frames)), config);

  const auto first = recognizer.Recognize({});
  RGBD_CHECK(first.empty());
  const auto second = recognizer.Recognize({});
  RGBD_CHECK(second.size() == 1);
  RGBD_CHECK(second[0].box.x == 12);
  RGBD_CHECK(second[0].box.width == 20);
  const auto third = recognizer.Recognize({});
  RGBD_CHECK(third.empty());
}

RGBD_TEST(detection_filter_drops_missed_tracks) {
  std::vector<std::vector<rgbd::Detection2D>> frames{
      {Detection(10)}, {Detection(10)}, {}, {},
      {Detection(40)}, {Detection(40)}};
  rgbd::DetectionFilterConfig config;
  config.min_hits = 2;
  config.max_missed = 1;
  config.min_iou = 0.1F;
  rgbd::FilteringRecognizer recognizer(
      std::make_unique<SequenceRecognizer>(std::move(frames)), config);

  RGBD_CHECK(recognizer.Recognize({}).empty());
  RGBD_CHECK(recognizer.Recognize({}).size() == 1);
  RGBD_CHECK(recognizer.Recognize({}).empty());
  RGBD_CHECK(recognizer.Recognize({}).empty());
  RGBD_CHECK(recognizer.Recognize({}).empty());
  RGBD_CHECK(recognizer.Recognize({}).size() == 1);
}

RGBD_TEST(detection_filter_matches_by_label_and_iou) {
  std::vector<std::vector<rgbd::Detection2D>> frames{
      {Detection(10)},
      {Detection(10)},
      {{"person", 0.8F, {10, 10, 20, 20}, std::nullopt}},
      {{"person", 0.8F, {10, 10, 20, 20}, std::nullopt}}};
  rgbd::DetectionFilterConfig config;
  config.min_hits = 2;
  config.min_iou = 0.1F;
  rgbd::FilteringRecognizer recognizer(
      std::make_unique<SequenceRecognizer>(std::move(frames)), config);

  RGBD_CHECK(recognizer.Recognize({}).empty());
  RGBD_CHECK(recognizer.Recognize({}).size() == 1);
  RGBD_CHECK(recognizer.Recognize({}).empty());
  const auto person = recognizer.Recognize({});
  RGBD_CHECK(person.size() == 1);
  RGBD_CHECK(person[0].label == "person");
}

RGBD_TEST(detection_filter_validates_config) {
  RGBD_CHECK_THROW(rgbd::FilteringRecognizer(nullptr), std::invalid_argument);
  rgbd::DetectionFilterConfig config;
  config.min_hits = 0;
  RGBD_CHECK_THROW(rgbd::FilteringRecognizer(
                       std::make_unique<SequenceRecognizer>(
                           std::vector<std::vector<rgbd::Detection2D>>{}),
                       config),
                   std::invalid_argument);
}
