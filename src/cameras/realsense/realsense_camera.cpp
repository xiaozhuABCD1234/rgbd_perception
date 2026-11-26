#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <librealsense2/rs.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rgbd/cameras/realsense.hpp"
#include "rgbd/core/frame.hpp"

namespace rgbd {

namespace {

constexpr std::size_t kDepthSampleBytes = sizeof(std::uint16_t);

std::string DeviceSerial(const rs2::device& device) {
  if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
    return {};
  }
  return device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
}

void CopyColor(const rs2::video_frame& source, ColorImage& target) {
  const auto width = static_cast<std::size_t>(source.get_width());
  const auto height = static_cast<std::size_t>(source.get_height());
  const auto size = width * height * ColorImage::channels;
  const auto* data = static_cast<const std::uint8_t*>(source.get_data());
  if (data == nullptr ||
      static_cast<std::size_t>(source.get_data_size()) < size) {
    throw std::runtime_error("realsense: truncated color frame");
  }
  target.width = width;
  target.height = height;
  target.pixels.assign(data, data + size);
}

void CopyDepth(const rs2::depth_frame& source, DepthImage& target) {
  const auto width = static_cast<std::size_t>(source.get_width());
  const auto height = static_cast<std::size_t>(source.get_height());
  const auto count = width * height;
  const auto* data = static_cast<const std::uint8_t*>(source.get_data());
  if (data == nullptr || static_cast<std::size_t>(source.get_data_size()) <
                             count * kDepthSampleBytes) {
    throw std::runtime_error("realsense: truncated depth frame");
  }
  const float depth_units = source.get_units();
  target.width = width;
  target.height = height;
  target.values.resize(count);
  for (std::size_t index = 0; index < count; ++index) {
    std::uint16_t raw = 0;
    std::memcpy(&raw, data + index * kDepthSampleBytes, kDepthSampleBytes);
    target.values[index] = static_cast<float>(raw) * depth_units;
  }
}

CameraIntrinsics IntrinsicsOf(const rs2::video_frame& frame) {
  const rs2::video_stream_profile profile =
      frame.get_profile().as<rs2::video_stream_profile>();
  const rs2_intrinsics intrinsics = profile.get_intrinsics();
  return {static_cast<std::size_t>(intrinsics.width),
          static_cast<std::size_t>(intrinsics.height),
          intrinsics.fx,
          intrinsics.fy,
          intrinsics.ppx,
          intrinsics.ppy};
}

class RealSenseCamera final : public IDepthCamera {
 public:
  explicit RealSenseCamera(RealSenseCameraConfig config)
      : config_(std::move(config)) {
    if (config_.camera_id.empty() || config_.depth_width == 0 ||
        config_.depth_height == 0 || config_.depth_fps <= 0 ||
        config_.color_width == 0 || config_.color_height == 0 ||
        config_.color_fps <= 0) {
      throw std::invalid_argument(
          "realsense camera requires an ID and valid stream profiles");
    }
  }

  ~RealSenseCamera() override { Close(); }

  void Open() override {
    if (pipeline_ != nullptr) {
      return;
    }
    if (!config_.serial.empty()) {
      bool found = false;
      for (const auto& device : context_.query_devices()) {
        if (DeviceSerial(device) == config_.serial) {
          found = true;
          break;
        }
      }
      if (!found) {
        throw std::runtime_error("realsense: no matching device");
      }
    }
    pipeline_ = std::make_unique<rs2::pipeline>(context_);
    stream_config_ = std::make_unique<rs2::config>();
    if (!config_.serial.empty()) {
      stream_config_->enable_device(config_.serial);
    }
    stream_config_->enable_stream(RS2_STREAM_COLOR,
                                  static_cast<int>(config_.color_width),
                                  static_cast<int>(config_.color_height),
                                  RS2_FORMAT_RGB8, config_.color_fps);
    stream_config_->enable_stream(RS2_STREAM_DEPTH,
                                  static_cast<int>(config_.depth_width),
                                  static_cast<int>(config_.depth_height),
                                  RS2_FORMAT_Z16, config_.depth_fps);
    if (config_.align_depth_to_color) {
      align_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
    }
    serial_ = config_.serial;
  }

  void Start() override {
    if (pipeline_ == nullptr) {
      throw std::logic_error("camera must be opened before start");
    }
    if (streaming_) {
      return;
    }
    pipeline_->start(*stream_config_);
    streaming_ = true;
  }

  void Stop() override {
    if (streaming_ && pipeline_ != nullptr) {
      pipeline_->stop();
    }
    streaming_ = false;
  }

  void Close() override {
    Stop();
    align_.reset();
    stream_config_.reset();
    pipeline_.reset();
  }

  bool Read(FrameSet& frame, std::chrono::milliseconds timeout) override {
    if (!streaming_ || pipeline_ == nullptr || timeout.count() <= 0) {
      return false;
    }
    rs2::frameset frames;
    try {
      frames = pipeline_->wait_for_frames(
          static_cast<unsigned int>(timeout.count()));
    } catch (const rs2::error&) {
      return false;
    }
    if (!frames) {
      return false;
    }
    const rs2::frameset aligned =
        align_ == nullptr ? frames : align_->process(frames);
    const rs2::video_frame color = aligned.get_color_frame();
    const rs2::depth_frame depth = aligned.get_depth_frame();
    if (!color || !depth) {
      return false;
    }
    Fill(color, depth, frame);
    return true;
  }

  [[nodiscard]] std::string Name() const override {
    return serial_.empty() ? config_.camera_id
                           : config_.camera_id + ":" + serial_;
  }

 private:
  void Fill(const rs2::video_frame& color, const rs2::depth_frame& depth,
            FrameSet& frame) const {
    frame = FrameSet{};
    frame.camera_id = config_.camera_id;
    frame.sequence = color.get_frame_number();
    frame.device_timestamp = Nanoseconds(
        static_cast<std::int64_t>(color.get_timestamp() * 1000000.0));
    frame.host_timestamp = std::chrono::duration_cast<Nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
    CopyColor(color, frame.color);
    CopyDepth(depth, frame.depth);
    frame.color_intrinsics = IntrinsicsOf(color);
  }

  RealSenseCameraConfig config_;
  rs2::context context_;
  std::unique_ptr<rs2::pipeline> pipeline_;
  std::unique_ptr<rs2::config> stream_config_;
  std::unique_ptr<rs2::align> align_;
  std::string serial_;
  bool streaming_ = false;
};

}

std::unique_ptr<IDepthCamera> CreateRealSenseCamera(
    RealSenseCameraConfig config) {
  return std::make_unique<RealSenseCamera>(std::move(config));
}

}
