#pragma once

#include <string>
#include <vector>

#include "rgbd/frame.hpp"

namespace rgbd {

struct Rect {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

struct Detection2D {
  std::string label;
  float confidence = 0.0F;
  Rect box;
};

class IObjectDetector {
 public:
  virtual ~IObjectDetector() = default;

  [[nodiscard]] virtual std::vector<Detection2D> detect(
      const ColorImage& color) = 0;
};

}  // namespace rgbd
