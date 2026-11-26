#include "rgbd/io/mcap_image_writer.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <mcap/writer.hpp>

#include "foxglove_messages.hpp"

namespace rgbd {

namespace {

void EnsureStatus(const mcap::Status& status) {
  if (status.code != mcap::StatusCode::Success) {
    throw std::runtime_error(status.message);
  }
}

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}

class McapImageWriter::Impl {
 public:
  explicit Impl(McapImageWriterConfig writer_config)
      : config(std::move(writer_config)) {
    if (config.path.empty()) {
      throw std::invalid_argument("MCAP path must not be empty");
    }
    raw_descriptor_data = internal::FoxgloveRawImageDescriptor();
    calibration_descriptor_data =
        internal::FoxgloveCameraCalibrationDescriptor();
    raw_schema =
        mcap::Schema("foxglove.RawImage", "protobuf", raw_descriptor_data);
    calibration_schema = mcap::Schema("foxglove.CameraCalibration", "protobuf",
                                      calibration_descriptor_data);
    mcap::McapWriterOptions options("rgbd_perception");
    options.noChunking = true;
    options.compression = mcap::Compression::None;
    EnsureStatus(writer.open(config.path, options));
    writer.addSchema(raw_schema);
    writer.addSchema(calibration_schema);

    color_channel.topic = config.color_topic;
    color_channel.messageEncoding = "protobuf";
    color_channel.schemaId = raw_schema.id;
    writer.addChannel(color_channel);
    depth_channel.topic = config.depth_topic;
    depth_channel.messageEncoding = "protobuf";
    depth_channel.schemaId = raw_schema.id;
    writer.addChannel(depth_channel);
    calibration_channel.topic = config.calibration_topic;
    calibration_channel.messageEncoding = "protobuf";
    calibration_channel.schemaId = calibration_schema.id;
    writer.addChannel(calibration_channel);
  }

  void Write(const FrameSet& frame) {
    if (!open) {
      throw std::logic_error("MCAP writer is closed");
    }
    if (!frame.Valid()) {
      throw std::invalid_argument("cannot write an invalid frame");
    }
    const std::int64_t timestamp = NowNs();
    const std::string color_payload = internal::SerializeFoxgloveRawImage(
        frame, timestamp, config.frame_id, "rgb8",
        static_cast<std::uint32_t>(frame.color.width * ColorImage::channels),
        frame.color.pixels.data(), frame.color.pixels.size());
    WriteMessage(color_channel.id, frame.sequence, timestamp, color_payload);

    std::vector<std::uint16_t> depth_mm(frame.depth.values.size());
    for (std::size_t index = 0; index < frame.depth.values.size(); ++index) {
      const float meters = frame.depth.values[index];
      std::uint16_t value = 0;
      if (std::isfinite(meters) && meters >= config.min_depth_m &&
          meters <= config.max_depth_m) {
        const float scaled = meters * 1000.0F;
        value = scaled >= static_cast<float>(
                              std::numeric_limits<std::uint16_t>::max())
                    ? std::numeric_limits<std::uint16_t>::max()
                    : static_cast<std::uint16_t>(scaled + 0.5F);
      }
      depth_mm[index] = value;
    }
    const std::string depth_payload = internal::SerializeFoxgloveRawImage(
        frame, timestamp, config.frame_id, "mono16",
        static_cast<std::uint32_t>(frame.depth.width * sizeof(std::uint16_t)),
        depth_mm.data(), depth_mm.size() * sizeof(std::uint16_t));
    WriteMessage(depth_channel.id, frame.sequence, timestamp, depth_payload);

    const std::string calibration_payload =
        internal::SerializeFoxgloveCameraCalibration(frame, timestamp,
                                                     config.frame_id);
    WriteMessage(calibration_channel.id, frame.sequence, timestamp,
                 calibration_payload);
    ++frame_count;
  }

  void Close() {
    if (open) {
      writer.close();
      open = false;
    }
  }

  void WriteMessage(mcap::ChannelId channel_id, std::uint64_t sequence,
                    std::int64_t timestamp, const std::string& payload) {
    mcap::Message record;
    record.channelId = channel_id;
    record.sequence = static_cast<std::uint32_t>(sequence);
    record.logTime = static_cast<mcap::Timestamp>(timestamp);
    record.publishTime = static_cast<mcap::Timestamp>(timestamp);
    record.dataSize = payload.size();
    record.data = reinterpret_cast<const std::byte*>(payload.data());
    EnsureStatus(writer.write(record));
  }

  McapImageWriterConfig config;
  mcap::McapWriter writer;
  std::string raw_descriptor_data;
  std::string calibration_descriptor_data;
  mcap::Schema raw_schema;
  mcap::Schema calibration_schema;
  mcap::Channel color_channel;
  mcap::Channel depth_channel;
  mcap::Channel calibration_channel;
  bool open = true;
  std::size_t frame_count = 0;
};

McapImageWriter::McapImageWriter(McapImageWriterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

McapImageWriter::~McapImageWriter() {
  if (impl_ != nullptr) {
    try {
      impl_->Close();
    } catch (...) {
    }
  }
}

void McapImageWriter::Write(const FrameSet& frame) { impl_->Write(frame); }

void McapImageWriter::Close() { impl_->Close(); }

std::size_t McapImageWriter::FrameCount() const noexcept {
  return impl_ == nullptr ? 0 : impl_->frame_count;
}

const std::string& McapImageWriter::Path() const noexcept {
  static const std::string kEmpty;
  return impl_ == nullptr ? kEmpty : impl_->config.path;
}

}
