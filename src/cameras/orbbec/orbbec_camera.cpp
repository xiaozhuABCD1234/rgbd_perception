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

int DisparitySearchMode(int range) {
  if (range == 64) {
    return 0;
  }
  if (range == 128) {
    return 1;
  }
  if (range == 256) {
    return 2;
  }
  throw std::invalid_argument("disparity_search_range must be 64, 128 or 256");
}

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
    if (config_.camera_id.empty() || config_.depth_width == 0 ||
        config_.depth_height == 0 || config_.depth_fps <= 0 ||
        config_.color_width == 0 || config_.color_height == 0 ||
        config_.color_fps <= 0) {
      throw std::invalid_argument(
          "orbbec camera requires an ID and valid stream profiles");
    }
    if (config_.depth_exposure_us <= 0 || config_.depth_gain <= 0 ||
        config_.ae_max_exposure_us <= 0) {
      throw std::invalid_argument("orbbec exposure and gain must be positive");
    }
    DisparitySearchMode(config_.disparity_search_range);
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
    stream_config_->enableVideoStream(
        OB_STREAM_COLOR, static_cast<std::uint32_t>(config_.color_width),
        static_cast<std::uint32_t>(config_.color_height),
        static_cast<std::uint32_t>(config_.color_fps), OB_FORMAT_RGB);
    stream_config_->enableVideoStream(
        OB_STREAM_DEPTH, static_cast<std::uint32_t>(config_.depth_width),
        static_cast<std::uint32_t>(config_.depth_height),
        static_cast<std::uint32_t>(config_.depth_fps), OB_FORMAT_Y16);
    if (pipeline_->getStreamProfileList(OB_SENSOR_COLOR)->getCount() == 0 ||
        pipeline_->getStreamProfileList(OB_SENSOR_DEPTH)->getCount() == 0) {
      throw std::runtime_error("orbbec: device exposes no video profiles");
    }
    if (config_.align_depth_to_color) {
      align_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
    }
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
    try {
      ApplyControls();
    } catch (...) {
      Stop();
      throw;
    }
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
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) {
        break;
      }
      std::shared_ptr<ob::FrameSet> frames;
      try {
        const auto raw = pipeline_->waitForFrameset(
            static_cast<std::uint32_t>(remaining.count()));
        if (raw == nullptr) {
          continue;
        }
        const auto raw_color = raw->getColorFrame();
        const auto raw_depth = raw->getDepthFrame();
        if (raw_color == nullptr || raw_depth == nullptr) {
          continue;
        }
        if (align_ != nullptr) {
          const auto processed = align_->process(raw);
          if (processed == nullptr) {
            continue;
          }
          frames = processed->as<ob::FrameSet>();
        } else {
          frames = raw;
        }
      } catch (const ob::Error&) {
        continue;
      }
      if (frames == nullptr) {
        continue;
      }
      const auto color = frames->getColorFrame();
      const auto depth = frames->getDepthFrame();
      if (color == nullptr || depth == nullptr) {
        continue;
      }
      if (color->getFormat() != OB_FORMAT_RGB ||
          depth->getFormat() != OB_FORMAT_Y16) {
        throw std::runtime_error("orbbec: unexpected frame format");
      }
      Fill(*color, *depth, frame);
      return true;
    }
    return false;
  }

  [[nodiscard]] std::string Name() const override {
    return serial_.empty() ? config_.camera_id
                           : config_.camera_id + ":" + serial_;
  }

 private:
  void SetIntChecked(OBPropertyID property, int value, const char* name) const {
    if (!device_->isPropertySupported(property, OB_PERMISSION_WRITE)) {
      throw std::runtime_error(std::string(name) + " is not supported");
    }
    const OBIntPropertyRange range = device_->getIntPropertyRange(property);
    if (value < range.min || value > range.max) {
      throw std::invalid_argument(std::string(name) + " must be in [" +
                                  std::to_string(range.min) + ", " +
                                  std::to_string(range.max) + "]");
    }
    device_->setIntProperty(property, value);
  }

  void ApplyControls() const {
    if (!device_->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL,
                                      OB_PERMISSION_WRITE)) {
      throw std::runtime_error("depth auto exposure is not supported");
    }
    device_->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL,
                             config_.depth_auto_exposure);
    if (config_.depth_auto_exposure) {
      SetIntChecked(OB_PROP_IR_AE_MAX_EXPOSURE_INT, config_.ae_max_exposure_us,
                    "ae_max_exposure_us");
    } else {
      SetIntChecked(OB_PROP_DEPTH_EXPOSURE_INT, config_.depth_exposure_us,
                    "exposure_us");
      SetIntChecked(OB_PROP_DEPTH_GAIN_INT, config_.depth_gain, "gain");
    }
    if (!device_->isPropertySupported(OB_PROP_DISP_SEARCH_RANGE_MODE_INT,
                                      OB_PERMISSION_WRITE)) {
      throw std::runtime_error("disparity search range is not supported");
    }
    const int mode = DisparitySearchMode(config_.disparity_search_range);
    device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, mode);
    if (device_->getIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT) != mode) {
      throw std::runtime_error("failed to apply disparity search range");
    }
  }

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
