#include "rgbd/io/mcap_image_writer.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <mcap/writer.hpp>

#include "foxglove/CameraCalibration.pb.h"
#include "foxglove/RawImage.pb.h"

namespace rgbd {

namespace {

void AddFileDescriptor(const google::protobuf::FileDescriptor* file,
                       std::set<std::string>* added,
                       google::protobuf::FileDescriptorSet* set) {
  if (file == nullptr || added->count(file->name()) != 0) {
    return;
  }
  added->insert(file->name());
  for (int index = 0; index < file->dependency_count(); ++index) {
    AddFileDescriptor(file->dependency(index), added, set);
  }
  google::protobuf::FileDescriptorProto* proto = set->add_file();
  file->CopyTo(proto);
}

std::string DescriptorSet(const google::protobuf::Descriptor* descriptor) {
  google::protobuf::FileDescriptorSet set;
  std::set<std::string> added;
  AddFileDescriptor(descriptor->file(), &added, &set);
  std::string serialized;
  if (!set.SerializeToString(&serialized)) {
    throw std::runtime_error("failed to serialize protobuf descriptor set");
  }
  return serialized;
}

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

void SetTimestamp(std::int64_t nanoseconds,
                  google::protobuf::Timestamp* timestamp) {
  std::int64_t seconds = nanoseconds / 1000000000;
  std::int64_t nanos = nanoseconds % 1000000000;
  if (nanos < 0) {
    nanos += 1000000000;
    --seconds;
  }
  timestamp->set_seconds(seconds);
  timestamp->set_nanos(static_cast<std::int32_t>(nanos));
}

}

class McapImageWriter::Impl {
 public:
  explicit Impl(McapImageWriterConfig writer_config)
      : config(std::move(writer_config)) {
    if (config.path.empty()) {
      throw std::invalid_argument("MCAP path must not be empty");
    }
    raw_descriptor_data = DescriptorSet(foxglove::RawImage::descriptor());
    calibration_descriptor_data =
        DescriptorSet(foxglove::CameraCalibration::descriptor());
    raw_schema = mcap::Schema(foxglove::RawImage::descriptor()->full_name(),
                              "protobuf", raw_descriptor_data);
    calibration_schema =
        mcap::Schema(foxglove::CameraCalibration::descriptor()->full_name(),
                     "protobuf", calibration_descriptor_data);
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
    WriteImage(
        frame, timestamp, color_channel.id, "rgb8",
        static_cast<std::uint32_t>(frame.color.width * ColorImage::channels),
        frame.color.pixels.data(), frame.color.pixels.size());
    std::vector<std::uint16_t> depth_mm(frame.depth.values.size());
    for (std::size_t index = 0; index < frame.depth.values.size(); ++index) {
      const float meters = frame.depth.values[index];
      std::uint16_t value = 0;
      if (std::isfinite(meters) && meters > 0.0F) {
        const float scaled = meters * 1000.0F;
        value = scaled >= static_cast<float>(
                              std::numeric_limits<std::uint16_t>::max())
                    ? std::numeric_limits<std::uint16_t>::max()
                    : static_cast<std::uint16_t>(scaled + 0.5F);
      }
      depth_mm[index] = value;
    }
    WriteImage(
        frame, timestamp, depth_channel.id, "mono16",
        static_cast<std::uint32_t>(frame.depth.width * sizeof(std::uint16_t)),
        depth_mm.data(), depth_mm.size() * sizeof(std::uint16_t));
    if (!calibration_written) {
      WriteCalibration(frame, timestamp);
      calibration_written = true;
    }
    ++frame_count;
  }

  void Close() {
    if (open) {
      writer.close();
      open = false;
    }
  }

  void WriteImage(const FrameSet& frame, std::int64_t timestamp,
                  mcap::ChannelId channel_id, const std::string& encoding,
                  std::uint32_t step, const void* data, std::size_t size) {
    foxglove::RawImage image;
    SetTimestamp(timestamp, image.mutable_timestamp());
    image.set_frame_id(config.frame_id);
    image.set_width(static_cast<std::uint32_t>(frame.color.width));
    image.set_height(static_cast<std::uint32_t>(frame.color.height));
    image.set_encoding(encoding);
    image.set_step(step);
    image.set_data(data, size);
    WriteMessage(channel_id, frame.sequence, timestamp, image);
  }

  void WriteCalibration(const FrameSet& frame, std::int64_t timestamp) {
    foxglove::CameraCalibration calibration;
    SetTimestamp(timestamp, calibration.mutable_timestamp());
    calibration.set_frame_id(config.frame_id);
    calibration.set_width(static_cast<std::uint32_t>(frame.color.width));
    calibration.set_height(static_cast<std::uint32_t>(frame.color.height));
    calibration.set_distortion_model("plumb_bob");
    calibration.add_d(0.0);
    calibration.add_d(0.0);
    calibration.add_d(0.0);
    calibration.add_d(0.0);
    calibration.add_d(0.0);
    const CameraIntrinsics& intrinsics = frame.color_intrinsics;
    for (const double value :
         {static_cast<double>(intrinsics.fx), 0.0,
          static_cast<double>(intrinsics.cx), 0.0,
          static_cast<double>(intrinsics.fy),
          static_cast<double>(intrinsics.cy), 0.0, 0.0, 1.0}) {
      calibration.add_k(value);
    }
    for (const double value : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {
      calibration.add_r(value);
    }
    for (const double value :
         {static_cast<double>(intrinsics.fx), 0.0,
          static_cast<double>(intrinsics.cx), 0.0, 0.0,
          static_cast<double>(intrinsics.fy),
          static_cast<double>(intrinsics.cy), 0.0, 0.0, 0.0, 1.0, 0.0}) {
      calibration.add_p(value);
    }
    WriteMessage(calibration_channel.id, frame.sequence, timestamp,
                 calibration);
  }

  template <typename Message>
  void WriteMessage(mcap::ChannelId channel_id, std::uint64_t sequence,
                    std::int64_t timestamp, const Message& message) {
    std::string payload;
    if (!message.SerializeToString(&payload)) {
      throw std::runtime_error("failed to serialize Foxglove message");
    }
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
  bool calibration_written = false;
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
