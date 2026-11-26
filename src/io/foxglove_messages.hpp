#ifndef RGBD_IO_FOXGLOVE_MESSAGES_HPP_
#define RGBD_IO_FOXGLOVE_MESSAGES_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rgbd/core/frame.hpp"

namespace rgbd {
namespace internal {

std::string FoxgloveRawImageDescriptor();
std::string FoxgloveCameraCalibrationDescriptor();
std::string SerializeFoxgloveRawImage(const FrameSet& frame,
                                      std::int64_t timestamp,
                                      const std::string& frame_id,
                                      const std::string& encoding,
                                      std::uint32_t step, const void* data,
                                      std::size_t size);
std::string SerializeFoxgloveCameraCalibration(const FrameSet& frame,
                                               std::int64_t timestamp,
                                               const std::string& frame_id);

}
}

#endif
