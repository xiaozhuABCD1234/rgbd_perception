#ifndef RGBD_RECOGNITION_YOLO26_RECOGNIZER_HPP_
#define RGBD_RECOGNITION_YOLO26_RECOGNIZER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rgbd/core/recognizer.hpp"
#include "rgbd/inference/engine.hpp"

namespace rgbd {

struct Yolo26Config {
  std::string input_name = "images";
  std::vector<std::string> labels;
  int input_width = 640;
  int input_height = 640;
  float confidence_threshold = 0.25F;
  float iou_threshold = 0.45F;
  float mask_threshold = 0.5F;
};

class Yolo26Recognizer final : public IVisionRecognizer {
 public:
  explicit Yolo26Recognizer(std::shared_ptr<IInferenceEngine> engine,
                            Yolo26Config config = {});
  ~Yolo26Recognizer() override;

  Yolo26Recognizer(const Yolo26Recognizer&) = delete;
  Yolo26Recognizer& operator=(const Yolo26Recognizer&) = delete;

  [[nodiscard]] std::vector<Detection2D> Recognize(
      const FrameSet& frame) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rgbd

#endif
