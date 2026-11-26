#ifndef RGBD_CORE_RECOGNIZER_HPP_
#define RGBD_CORE_RECOGNIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rgbd/core/frame.hpp"

namespace rgbd {

struct Rect {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;

  [[nodiscard]] bool Valid() const noexcept { return width > 0 && height > 0; }
};

struct Mask2D {
  std::size_t width = 0;
  std::size_t height = 0;
  std::vector<std::uint8_t> values;

  [[nodiscard]] bool Valid() const noexcept {
    return width > 0 && height > 0 && values.size() == width * height;
  }
};

struct Detection2D {
  std::string label;
  float confidence = 0.0F;
  Rect box;
  std::optional<Mask2D> mask;
};

class IVisionRecognizer {
 public:
  virtual ~IVisionRecognizer() = default;

  [[nodiscard]] virtual std::vector<Detection2D> Recognize(
      const FrameSet& frame) = 0;
};

}

#endif
