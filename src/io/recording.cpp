#include "rgbd/io/recording.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

constexpr std::array<char, 8> kMagic{{'R', 'G', 'B', 'D', 'R', 'E', 'C', '1'}};
constexpr std::uint32_t kVersion = 1;
constexpr std::uint32_t kMaxStringBytes = 4096;
constexpr std::uint64_t kMaxPayloadBytes = 1ULL << 30;

void WriteBytes(std::ostream& stream, const void* data, std::size_t size) {
  if (size == 0) {
    return;
  }
  stream.write(static_cast<const char*>(data),
               static_cast<std::streamsize>(size));
  if (!stream) {
    throw std::runtime_error("failed to write recording");
  }
}

void WriteU32(std::ostream& stream, std::uint32_t value) {
  std::array<std::uint8_t, 4> bytes{};
  bytes[0] = static_cast<std::uint8_t>(value & 0xFFU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  bytes[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  bytes[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
  WriteBytes(stream, bytes.data(), bytes.size());
}

void WriteU64(std::ostream& stream, std::uint64_t value) {
  std::array<std::uint8_t, 8> bytes{};
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xFFU);
  }
  WriteBytes(stream, bytes.data(), bytes.size());
}

void WriteI64(std::ostream& stream, std::int64_t value) {
  WriteU64(stream, static_cast<std::uint64_t>(value));
}

void WriteF32(std::ostream& stream, float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  WriteU32(stream, bits);
}

void WriteFloatArray(std::ostream& stream, const std::vector<float>& values) {
  std::vector<std::uint8_t> bytes(values.size() * sizeof(float));
  for (std::size_t index = 0; index < values.size(); ++index) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &values[index], sizeof(bits));
    bytes[index * 4] = static_cast<std::uint8_t>(bits & 0xFFU);
    bytes[index * 4 + 1] = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
    bytes[index * 4 + 2] = static_cast<std::uint8_t>((bits >> 16U) & 0xFFU);
    bytes[index * 4 + 3] = static_cast<std::uint8_t>((bits >> 24U) & 0xFFU);
  }
  WriteBytes(stream, bytes.data(), bytes.size());
}

void WriteHeader(std::ostream& stream) {
  WriteBytes(stream, kMagic.data(), kMagic.size());
  WriteU32(stream, kVersion);
  WriteU32(stream, 0);
}

void WriteFrame(std::ostream& stream, const FrameSet& frame) {
  if (!frame.Valid()) {
    throw std::invalid_argument("cannot record an invalid frame");
  }
  const auto camera_id_size =
      static_cast<std::uint32_t>(frame.camera_id.size());
  WriteU32(stream, camera_id_size);
  WriteBytes(stream, frame.camera_id.data(), frame.camera_id.size());
  WriteU64(stream, frame.sequence);
  WriteI64(stream, frame.device_timestamp.count());
  WriteI64(stream, frame.host_timestamp.count());
  WriteU32(stream, static_cast<std::uint32_t>(frame.color.width));
  WriteU32(stream, static_cast<std::uint32_t>(frame.color.height));
  WriteU32(stream, static_cast<std::uint32_t>(frame.depth.width));
  WriteU32(stream, static_cast<std::uint32_t>(frame.depth.height));
  WriteF32(stream, frame.color_intrinsics.fx);
  WriteF32(stream, frame.color_intrinsics.fy);
  WriteF32(stream, frame.color_intrinsics.cx);
  WriteF32(stream, frame.color_intrinsics.cy);
  WriteU64(stream, static_cast<std::uint64_t>(frame.color.pixels.size()));
  WriteU64(stream, static_cast<std::uint64_t>(frame.depth.values.size()));
  WriteBytes(stream, frame.color.pixels.data(), frame.color.pixels.size());
  WriteFloatArray(stream, frame.depth.values);
}

}

class RecordingWriter::Impl {
 public:
  explicit Impl(std::string recording_path) : path(std::move(recording_path)) {
    stream.open(this->path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("failed to open recording for writing");
    }
    WriteHeader(stream);
  }

  void Write(const FrameSet& frame) {
    if (!stream.is_open()) {
      throw std::logic_error("recording is closed");
    }
    WriteFrame(stream, frame);
    ++frame_count;
  }

  void Close() {
    if (stream.is_open()) {
      stream.flush();
      stream.close();
      if (!stream) {
        throw std::runtime_error("failed to finalize recording");
      }
    }
  }

  std::string path;
  std::ofstream stream;
  std::size_t frame_count = 0;
};

RecordingWriter::RecordingWriter(std::string path)
    : impl_(std::make_unique<Impl>(std::move(path))) {}

RecordingWriter::~RecordingWriter() {
  if (impl_ != nullptr) {
    try {
      impl_->Close();
    } catch (...) {
    }
  }
}

void RecordingWriter::Write(const FrameSet& frame) { impl_->Write(frame); }

void RecordingWriter::Close() { impl_->Close(); }

std::size_t RecordingWriter::FrameCount() const noexcept {
  return impl_ == nullptr ? 0 : impl_->frame_count;
}

const std::string& RecordingWriter::Path() const noexcept {
  static const std::string kEmpty;
  return impl_ == nullptr ? kEmpty : impl_->path;
}

}
