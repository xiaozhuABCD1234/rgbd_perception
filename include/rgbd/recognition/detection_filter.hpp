#ifndef RGBD_RECOGNITION_DETECTION_FILTER_HPP_
#define RGBD_RECOGNITION_DETECTION_FILTER_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "rgbd/core/recognizer.hpp"

namespace rgbd {

struct DetectionFilterConfig {
  std::size_t min_hits = 2;
  std::size_t max_missed = 3;
  float smoothing = 0.35F;
  float min_iou = 0.3F;
  std::size_t max_tracks = 64;
};

class FilteringRecognizer final : public IVisionRecognizer {
 public:
  explicit FilteringRecognizer(std::unique_ptr<IVisionRecognizer> recognizer,
                               DetectionFilterConfig config = {});
  ~FilteringRecognizer() override;

  FilteringRecognizer(const FilteringRecognizer&) = delete;
  FilteringRecognizer& operator=(const FilteringRecognizer&) = delete;

  [[nodiscard]] std::vector<Detection2D> Recognize(
      const FrameSet& frame) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}

#endif
