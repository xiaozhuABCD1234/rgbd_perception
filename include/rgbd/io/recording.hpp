#ifndef RGBD_IO_RECORDING_HPP_
#define RGBD_IO_RECORDING_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "rgbd/core/frame.hpp"

namespace rgbd {

class RecordingWriter {
 public:
  explicit RecordingWriter(std::string path);
  ~RecordingWriter();

  RecordingWriter(const RecordingWriter&) = delete;
  RecordingWriter& operator=(const RecordingWriter&) = delete;
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
