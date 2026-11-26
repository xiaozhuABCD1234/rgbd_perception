#include "foxglove_messages.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>

#include "foxglove/CameraCalibration.pb.h"
#include "foxglove/RawImage.pb.h"

namespace rgbd {
namespace internal {

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

std::string FoxgloveRawImageDescriptor() {
  return DescriptorSet(foxglove::RawImage::descriptor());
}

std::string FoxgloveCameraCalibrationDescriptor() {
  return DescriptorSet(foxglove::CameraCalibration::descriptor());
}

std::string SerializeFoxgloveRawImage(const FrameSet& frame,
                                      std::int64_t timestamp,
                                      const std::string& frame_id,
                                      const std::string& encoding,
                                      std::uint32_t step, const void* data,
                                      std::size_t size) {
  foxglove::RawImage image;
  SetTimestamp(timestamp, image.mutable_timestamp());
  image.set_frame_id(frame_id);
  image.set_width(static_cast<std::uint32_t>(frame.color.width));
  image.set_height(static_cast<std::uint32_t>(frame.color.height));
  image.set_encoding(encoding);
  image.set_step(step);
  image.set_data(data, size);
  std::string payload;
  if (!image.SerializeToString(&payload)) {
    throw std::runtime_error("failed to serialize Foxglove RawImage");
  }
  return payload;
}

std::string SerializeFoxgloveCameraCalibration(const FrameSet& frame,
                                               std::int64_t timestamp,
                                               const std::string& frame_id) {
  foxglove::CameraCalibration calibration;
  SetTimestamp(timestamp, calibration.mutable_timestamp());
  calibration.set_frame_id(frame_id);
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
        static_cast<double>(intrinsics.fy), static_cast<double>(intrinsics.cy),
        0.0, 0.0, 1.0}) {
    calibration.add_k(value);
  }
  for (const double value : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {
    calibration.add_r(value);
  }
  for (const double value :
       {static_cast<double>(intrinsics.fx), 0.0,
        static_cast<double>(intrinsics.cx), 0.0, 0.0,
        static_cast<double>(intrinsics.fy), static_cast<double>(intrinsics.cy),
        0.0, 0.0, 0.0, 1.0, 0.0}) {
    calibration.add_p(value);
  }
  std::string payload;
  if (!calibration.SerializeToString(&payload)) {
    throw std::runtime_error("failed to serialize Foxglove CameraCalibration");
  }
  return payload;
}

}
}
