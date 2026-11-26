#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <libobsensor/ObSensor.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rgbd/cameras/orbbec.hpp"
#include "rgbd/core/frame.hpp"

namespace rgbd {

namespace {

constexpr float kMillimetresPerMetre = 1000.0F;
constexpr std::size_t kDepthSampleBytes = sizeof(std::uint16_t);

std::string DeviceSerial(const ob::Device& device) {
  const auto info = device.getDeviceInfo();
  if (info == nullptr) {
    return {};
  }
  const char* serial = info->getSerialNumber();
  return serial == nullptr ? std::string{} : std::string{serial};
}

void CopyColor(const ob::VideoFrame& source, ColorImage& target) {
  const auto width = static_cast<std::size_t>(source.getWidth());
  const auto height = static_cast<std::size_t>(source.getHeight());
  const auto size = width * height * ColorImage::channels;
  const auto* data = source.getData();
  if (data == nullptr ||
      static_cast<std::size_t>(source.getDataSize()) < size) {
    throw std::runtime_error("orbbec: truncated color frame");
  }
  target.width = width;
  target.height = height;
  target.pixels.assign(data, data + size);
}

void CopyDepth(const ob::VideoFrame& source, DepthImage& target) {
  const auto width = static_cast<std::size_t>(source.getWidth());
  const auto height = static_cast<std::size_t>(source.getHeight());
  const auto count = width * height;
  const auto* data = source.getData();
  if (data == nullptr || static_cast<std::size_t>(source.getDataSize()) <
                             count * kDepthSampleBytes) {
    throw std::runtime_error("orbbec: truncated depth frame");
  }
  target.width = width;
  target.height = height;
  target.values.resize(count);
  for (std::size_t index = 0; index < count; ++index) {
    std::uint16_t millimetres = 0;
    std::memcpy(&millimetres, data + index * kDepthSampleBytes,
                kDepthSampleBytes);
    target.values[index] =
        static_cast<float>(millimetres) / kMillimetresPerMetre;
  }
}

CameraIntrinsics IntrinsicsOf(const ob::VideoFrame& frame) {
  const auto profile = frame.getStreamProfile();
  if (profile == nullptr) {
    throw std::runtime_error("orbbec: missing color stream profile");
  }
  const auto value = profile->as<ob::VideoStreamProfile>()->getIntrinsic();
  return {static_cast<std::size_t>(frame.getWidth()),
          static_cast<std::size_t>(frame.getHeight()),
          value.fx,
          value.fy,
          value.cx,
          value.cy};
}

class OrbbecCamera final : public IDepthCamera {
 public:
  explicit OrbbecCamera(OrbbecCameraConfig config)
      : config_(std::move(config)) {
    if (config_.camera_id.empty() || config_.width == 0 ||
        config_.height == 0 || config_.fps <= 0) {
      throw std::invalid_argument(
          "orbbec camera requires an ID and a valid stream profile");
    }
  }

  ~OrbbecCamera() override { Close(); }

  void Open() override {
    if (pipeline_ != nullptr) {
      return;
    }
    context_ = std::make_shared<ob::Context>();
    device_ = SelectDevice();
    pipeline_ = std::make_shared<ob::Pipeline>(device_);
    stream_config_ = std::make_shared<ob::Config>();
    const auto width = static_cast<std::uint32_t>(config_.width);
    const auto height = static_cast<std::uint32_t>(config_.height);
    const auto fps = static_cast<std::uint32_t>(config_.fps);
    stream_config_->enableVideoStream(OB_STREAM_COLOR, width, height, fps,
                                      OB_FORMAT_RGB);
    stream_config_->enableVideoStream(OB_STREAM_DEPTH, width, height, fps,
                                      OB_FORMAT_Y16);
    if (pipeline_->getStreamProfileList(OB_SENSOR_COLOR)->getCount() == 0 ||
        pipeline_->getStreamProfileList(OB_SENSOR_DEPTH)->getCount() == 0) {
      throw std::runtime_error("orbbec: device exposes no video profiles");
    }
    align_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
    serial_ = DeviceSerial(*device_);
  }

  void Start() override {
    if (pipeline_ == nullptr) {
      throw std::logic_error("camera must be opened before start");
    }
    if (streaming_) {
      return;
    }
    pipeline_->start(stream_config_);
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
    device_.reset();
    context_.reset();
  }

  bool Read(FrameSet& frame, std::chrono::milliseconds timeout) override {
    if (!streaming_ || pipeline_ == nullptr || timeout.count() <= 0) {
      return false;
    }
    std::shared_ptr<ob::FrameSet> frames;
    try {
      const auto raw = pipeline_->waitForFrameset(
          static_cast<std::uint32_t>(timeout.count()));
      if (raw == nullptr) {
        return false;
      }
      const auto processed = align_->process(raw);
      if (processed == nullptr) {
        return false;
      }
      frames = processed->as<ob::FrameSet>();
    } catch (const ob::Error&) {
      return false;
    }
    if (frames == nullptr) {
      return false;
    }
    const auto color = frames->getColorFrame();
    const auto depth = frames->getDepthFrame();
    if (color == nullptr || depth == nullptr) {
      return false;
    }
    if (color->getFormat() != OB_FORMAT_RGB ||
        depth->getFormat() != OB_FORMAT_Y16) {
      throw std::runtime_error("orbbec: unexpected frame format");
    }
    Fill(*color, *depth, frame);
    return true;
  }

  [[nodiscard]] std::string Name() const override {
    return serial_.empty() ? config_.camera_id
                           : config_.camera_id + ":" + serial_;
  }

 private:
  std::shared_ptr<ob::Device> SelectDevice() const {
    auto list = context_->queryDeviceList();
    if (list == nullptr) {
      throw std::runtime_error("orbbec: device list unavailable");
    }
    const auto count = static_cast<std::size_t>(list->getCount());
    for (std::size_t index = 0; index < count; ++index) {
      auto device = list->getDevice(static_cast<std::uint32_t>(index));
      if (device == nullptr) {
        continue;
      }
      if (config_.serial.empty() || DeviceSerial(*device) == config_.serial) {
        return device;
      }
    }
    throw std::runtime_error("orbbec: no matching device");
  }

  void Fill(const ob::ColorFrame& color, const ob::DepthFrame& depth,
            FrameSet& frame) const {
    frame = FrameSet{};
    frame.camera_id = config_.camera_id;
    frame.sequence = color.getIndex();
    frame.device_timestamp =
        std::chrono::duration_cast<Nanoseconds>(std::chrono::microseconds(
            static_cast<std::int64_t>(color.getTimeStampUs())));
    frame.host_timestamp = std::chrono::duration_cast<Nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
    CopyColor(color, frame.color);
    CopyDepth(depth, frame.depth);
    frame.color_intrinsics = IntrinsicsOf(color);
  }

  OrbbecCameraConfig config_;
  std::shared_ptr<ob::Context> context_;
  std::shared_ptr<ob::Device> device_;
  std::shared_ptr<ob::Pipeline> pipeline_;
  std::shared_ptr<ob::Config> stream_config_;
  std::shared_ptr<ob::Align> align_;
  std::string serial_;
  bool streaming_ = false;
};

}

std::unique_ptr<IDepthCamera> CreateOrbbecCamera(OrbbecCameraConfig config) {
  return std::make_unique<OrbbecCamera>(std::move(config));
}

}
