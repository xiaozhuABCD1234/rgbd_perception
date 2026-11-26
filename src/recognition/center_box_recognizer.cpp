#include "rgbd/recognition/center_box_recognizer.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace rgbd {

CenterBoxRecognizer::CenterBoxRecognizer(CenterBoxRecognizerConfig config)
    : config_(config) {
  if (config_.label.empty()) {
    throw std::invalid_argument("label must not be empty");
  }
  if (config_.scale <= 0.0F || config_.scale > 1.0F) {
    throw std::invalid_argument("scale must be in (0, 1]");
  }
  if (config_.confidence < 0.0F || config_.confidence > 1.0F) {
    throw std::invalid_argument("confidence must be in [0, 1]");
  }
}

std::vector<Detection2D> CenterBoxRecognizer::Recognize(const FrameSet& frame) {
  if (!frame.color.Valid()) {
    return {};
  }
  const auto width = static_cast<int>(frame.color.width);
  const auto height = static_cast<int>(frame.color.height);
  const int box_width =
      static_cast<int>(static_cast<float>(width) * config_.scale);
  const int box_height =
      static_cast<int>(static_cast<float>(height) * config_.scale);
  if (box_width <= 0 || box_height <= 0) {
    return {};
  }
  return {{config_.label,
           config_.confidence,
           {(width - box_width) / 2, (height - box_height) / 2, box_width,
            box_height},
           std::nullopt}};
}

}
