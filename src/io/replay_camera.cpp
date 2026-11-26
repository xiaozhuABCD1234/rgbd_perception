#include "rgbd/io/replay_camera.hpp"

#include <array>
#include <chrono>
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

void ReadBytes(std::istream& stream, void* data, std::size_t size) {
  if (size == 0) {
    return;
  }
  stream.read(static_cast<char*>(data), static_cast<std::streamsize>(size));
  if (!stream) {
    throw std::runtime_error("truncated recording");
  }
}

std::uint32_t ReadU32(std::istream& stream) {
  std::array<std::uint8_t, 4> bytes{};
  ReadBytes(stream, bytes.data(), bytes.size());
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

std::uint64_t ReadU64(std::istream& stream) {
  std::array<std::uint8_t, 8> bytes{};
  ReadBytes(stream, bytes.data(), bytes.size());
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    value |= static_cast<std::uint64_t>(bytes[index]) << (index * 8U);
  }
  return value;
}

std::int64_t ReadI64(std::istream& stream) {
  return static_cast<std::int64_t>(ReadU64(stream));
}

float ReadF32(std::istream& stream) {
  const std::uint32_t bits = ReadU32(stream);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void ReadFloatArray(std::istream& stream, std::size_t count,
                    std::vector<float>* values) {
  std::vector<std::uint8_t> bytes(count * sizeof(float));
  ReadBytes(stream, bytes.data(), bytes.size());
  values->resize(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::uint32_t bits =
        static_cast<std::uint32_t>(bytes[index * 4]) |
        (static_cast<std::uint32_t>(bytes[index * 4 + 1]) << 8U) |
        (static_cast<std::uint32_t>(bytes[index * 4 + 2]) << 16U) |
        (static_cast<std::uint32_t>(bytes[index * 4 + 3]) << 24U);
    std::memcpy(&(*values)[index], &bits, sizeof(bits));
  }
}

void ReadHeader(std::istream& stream) {
  std::array<char, 8> magic{};
  ReadBytes(stream, magic.data(), magic.size());
  if (magic != kMagic) {
    throw std::runtime_error("invalid recording magic");
  }
  if (ReadU32(stream) != kVersion) {
    throw std::runtime_error("unsupported recording version");
  }
  ReadU32(stream);
}

bool ReadFrame(std::istream& stream, FrameSet& frame) {
  if (stream.peek() == std::char_traits<char>::eof()) {
    return false;
  }
  const std::uint32_t camera_id_size = ReadU32(stream);
  if (camera_id_size > kMaxStringBytes) {
    throw std::runtime_error("invalid recording camera ID");
  }
  std::string camera_id(camera_id_size, '\0');
  ReadBytes(stream, camera_id.data(), camera_id.size());

  FrameSet loaded;
  loaded.camera_id = std::move(camera_id);
  loaded.sequence = ReadU64(stream);
  loaded.device_timestamp = Nanoseconds(ReadI64(stream));
  loaded.host_timestamp = Nanoseconds(ReadI64(stream));
  loaded.color.width = ReadU32(stream);
  loaded.color.height = ReadU32(stream);
  loaded.depth.width = ReadU32(stream);
  loaded.depth.height = ReadU32(stream);
  loaded.color_intrinsics = {
      loaded.color.width, loaded.color.height, ReadF32(stream),
      ReadF32(stream),    ReadF32(stream),     ReadF32(stream),
  };
  const std::uint64_t color_size = ReadU64(stream);
  const std::uint64_t depth_size = ReadU64(stream);
  if (color_size > kMaxPayloadBytes || depth_size > kMaxPayloadBytes) {
    throw std::runtime_error("invalid recording payload size");
  }
  if (loaded.color.width == 0 || loaded.color.height == 0 ||
      loaded.color.width > 16384 || loaded.color.height > 16384) {
    throw std::runtime_error("invalid recording dimensions");
  }
  const std::uint64_t expected_color = static_cast<std::uint64_t>(
      loaded.color.width * loaded.color.height * ColorImage::channels);
  const std::uint64_t expected_depth =
      static_cast<std::uint64_t>(loaded.depth.width * loaded.depth.height);
  if (color_size != expected_color || depth_size != expected_depth) {
    throw std::runtime_error("recording payload does not match dimensions");
  }
  loaded.color.pixels.resize(static_cast<std::size_t>(color_size));
  ReadBytes(stream, loaded.color.pixels.data(), loaded.color.pixels.size());
  ReadFloatArray(stream, static_cast<std::size_t>(depth_size),
                 &loaded.depth.values);
  if (!loaded.Valid()) {
    throw std::runtime_error("recording contains an invalid frame");
  }
  frame = std::move(loaded);
  return true;
}

}

class ReplayCamera::Impl {
 public:
  explicit Impl(ReplayCameraConfig camera_config)
      : config(std::move(camera_config)), data_start(0) {}

  void Open() {
    if (opened) {
      return;
    }
    stream.open(config.path, std::ios::binary);
    if (!stream) {
      throw std::runtime_error("failed to open recording");
    }
    ReadHeader(stream);
    data_start = stream.tellg();
    opened = true;
  }

  void Start() {
    if (!opened) {
      throw std::logic_error("camera must be opened before start");
    }
    if (!stream.is_open()) {
      stream.clear();
      stream.open(config.path, std::ios::binary);
      if (!stream) {
        throw std::runtime_error("failed to reopen recording");
      }
      ReadHeader(stream);
      data_start = stream.tellg();
    }
    streaming = true;
  }

  void Stop() { streaming = false; }

  void Close() {
    Stop();
    if (stream.is_open()) {
      stream.close();
    }
    opened = false;
  }

  bool Read(FrameSet& frame) {
    if (!streaming) {
      return false;
    }
    if (ReadFrame(stream, frame)) {
      return true;
    }
    if (!config.loop) {
      return false;
    }
    stream.clear();
    stream.seekg(data_start);
    if (!stream) {
      throw std::runtime_error("failed to rewind recording");
    }
    return ReadFrame(stream, frame);
  }

  ReplayCameraConfig config;
  std::ifstream stream;
  std::streampos data_start;
  bool opened = false;
  bool streaming = false;
};

ReplayCamera::ReplayCamera(ReplayCameraConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
  if (impl_->config.path.empty()) {
    throw std::invalid_argument("replay path must not be empty");
  }
  if (impl_->config.camera_id.empty()) {
    throw std::invalid_argument("replay camera ID must not be empty");
  }
}

ReplayCamera::~ReplayCamera() { impl_->Close(); }

void ReplayCamera::Open() { impl_->Open(); }

void ReplayCamera::Start() { impl_->Start(); }

void ReplayCamera::Stop() { impl_->Stop(); }

void ReplayCamera::Close() { impl_->Close(); }

bool ReplayCamera::Read(FrameSet& frame, std::chrono::milliseconds timeout) {
  static_cast<void>(timeout);
  return impl_->Read(frame);
}

std::string ReplayCamera::Name() const {
  return impl_->config.camera_id + ":" + impl_->config.path;
}

}
