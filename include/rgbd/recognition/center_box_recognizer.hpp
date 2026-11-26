#ifndef RGBD_RECOGNITION_CENTER_BOX_RECOGNIZER_HPP_
#define RGBD_RECOGNITION_CENTER_BOX_RECOGNIZER_HPP_

#include <string>
#include <vector>

#include "rgbd/core/recognizer.hpp"

namespace rgbd {

struct CenterBoxRecognizerConfig {
  std::string label = "object";
  float scale = 0.5F;
  float confidence = 1.0F;
};

class CenterBoxRecognizer final : public IVisionRecognizer {
 public:
  explicit CenterBoxRecognizer(CenterBoxRecognizerConfig config = {});

  [[nodiscard]] std::vector<Detection2D> Recognize(
      const FrameSet& frame) override;

 private:
  CenterBoxRecognizerConfig config_;
};

}

#endif
