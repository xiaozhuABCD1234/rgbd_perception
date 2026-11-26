#ifndef RGBD_IO_MCAP_IMAGE_WRITER_HPP_
#define RGBD_IO_MCAP_IMAGE_WRITER_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "rgbd/core/frame.hpp"

namespace rgbd {

struct McapImageWriterConfig {
  std::string path;
  std::string color_topic = "/camera/color/image_raw";
  std::string depth_topic = "/camera/depth/image_raw";
  std::string calibration_topic = "/camera/color/camera_info";
  std::string frame_id = "camera";
  float min_depth_m = 0.04F;
  float max_depth_m = 10.0F;
};

class McapImageWriter {
 public:
  explicit McapImageWriter(McapImageWriterConfig config);
  ~McapImageWriter();

  McapImageWriter(const McapImageWriter&) = delete;
  McapImageWriter& operator=(const McapImageWriter&) = delete;

  void Write(const FrameSet& frame);
  void Close();

  [[nodiscard]] std::size_t FrameCount() const noexcept;
  [[nodiscard]] const std::string& Path() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}

#endif
