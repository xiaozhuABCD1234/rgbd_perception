#include "foxglove_messages.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "foxglove/CameraCalibration.pb.h"
#include "foxglove/ImageAnnotations.pb.h"
#include "foxglove/RawImage.pb.h"
#include "foxglove/SceneUpdate.pb.h"

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

std::string FoxgloveSceneUpdateDescriptor() {
  return DescriptorSet(foxglove::SceneUpdate::descriptor());
}

std::string FoxgloveImageAnnotationsDescriptor() {
  return DescriptorSet(foxglove::ImageAnnotations::descriptor());
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

namespace {

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

Quaternion QuaternionFromRotation(const std::array<float, 9>& rotation) {
  const double m00 = rotation[0];
  const double m01 = rotation[1];
  const double m02 = rotation[2];
  const double m10 = rotation[3];
  const double m11 = rotation[4];
  const double m12 = rotation[5];
  const double m20 = rotation[6];
  const double m21 = rotation[7];
  const double m22 = rotation[8];
  const double trace = m00 + m11 + m22;
  Quaternion quaternion;
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * scale;
    quaternion.x = (m21 - m12) / scale;
    quaternion.y = (m02 - m20) / scale;
    quaternion.z = (m10 - m01) / scale;
  } else if (m00 > m11 && m00 > m22) {
    const double scale = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    quaternion.w = (m21 - m12) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (m01 + m10) / scale;
    quaternion.z = (m02 + m20) / scale;
  } else if (m11 > m22) {
    const double scale = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    quaternion.w = (m02 - m20) / scale;
    quaternion.x = (m01 + m10) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (m12 + m21) / scale;
  } else {
    const double scale = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    quaternion.w = (m10 - m01) / scale;
    quaternion.x = (m02 + m20) / scale;
    quaternion.y = (m12 + m21) / scale;
    quaternion.z = 0.25 * scale;
  }
  const double norm =
      std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm > 1e-12) {
    quaternion.x /= norm;
    quaternion.y /= norm;
    quaternion.z /= norm;
    quaternion.w /= norm;
  }
  return quaternion;
}

Point3f OpticalToDisplay(const Point3f& point) noexcept {
  return {point.z, -point.x, -point.y};
}

std::array<float, 9> RotationToDisplay(const std::array<float, 9>& rotation) {
  const auto to_display = [](float x, float y, float z) {
    return Point3f{z, -x, -y};
  };
  const Point3f x_axis = to_display(rotation[0], rotation[3], rotation[6]);
  const Point3f y_axis = to_display(rotation[1], rotation[4], rotation[7]);
  const Point3f z_axis = to_display(rotation[2], rotation[5], rotation[8]);
  return {x_axis.x, y_axis.x, z_axis.x, x_axis.y, y_axis.y,
          z_axis.y, x_axis.z, y_axis.z, z_axis.z};
}

void SetVector3(const Point3f& value, foxglove::Vector3* output) {
  output->set_x(value.x);
  output->set_y(value.y);
  output->set_z(value.z);
}

void SetColor(const std::array<double, 3>& color, double alpha,
              foxglove::Color* output) {
  output->set_r(color[0]);
  output->set_g(color[1]);
  output->set_b(color[2]);
  output->set_a(alpha);
}

std::string ConfidenceText(float confidence) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << confidence;
  return stream.str();
}

std::array<double, 3> ColorForLabel(const std::string& label) {
  std::uint32_t hash = 2166136261U;
  for (const char character : label) {
    hash ^= static_cast<std::uint8_t>(character);
    hash *= 16777619U;
  }
  static constexpr std::array<std::array<double, 3>, 6> kColors{{
      {{0.95, 0.32, 0.28}},
      {{0.24, 0.72, 0.45}},
      {{0.28, 0.52, 0.92}},
      {{0.93, 0.68, 0.22}},
      {{0.68, 0.40, 0.86}},
      {{0.20, 0.78, 0.82}},
  }};
  return kColors[hash % kColors.size()];
}

}  // namespace

std::string SerializeFoxgloveSceneUpdate(
    std::int64_t timestamp, const std::string& frame_id,
    const std::vector<ObjectObservation>& observations) {
  foxglove::SceneUpdate update;
  auto* deletion = update.add_deletions();
  SetTimestamp(timestamp > 0 ? timestamp - 1 : timestamp,
               deletion->mutable_timestamp());
  deletion->set_type(foxglove::SceneEntityDeletion_Type_ALL);

  std::size_t object_index = 0;
  for (const ObjectObservation& observation : observations) {
    if (!observation.geometry.has_value()) {
      continue;
    }
    const ObjectGeometry& geometry = *observation.geometry;
    const std::array<float, 9> rotation =
        RotationToDisplay(geometry.box.rotation);
    const Quaternion quaternion = QuaternionFromRotation(rotation);
    const Point3f center = OpticalToDisplay(geometry.box.center);

    auto* entity = update.add_entities();
    SetTimestamp(timestamp, entity->mutable_timestamp());
    entity->set_frame_id(frame_id);
    entity->set_id("object_" + std::to_string(object_index));
    entity->set_frame_locked(false);
    auto* label_metadata = entity->add_metadata();
    label_metadata->set_key("label");
    label_metadata->set_value(observation.detection.label);
    auto* confidence_metadata = entity->add_metadata();
    confidence_metadata->set_key("confidence");
    confidence_metadata->set_value(
        std::to_string(observation.detection.confidence));
    auto* points_metadata = entity->add_metadata();
    points_metadata->set_key("points");
    points_metadata->set_value(std::to_string(geometry.point_count));

    auto* cube = entity->add_cubes();
    SetVector3(center, cube->mutable_pose()->mutable_position());
    auto* orientation = cube->mutable_pose()->mutable_orientation();
    orientation->set_x(quaternion.x);
    orientation->set_y(quaternion.y);
    orientation->set_z(quaternion.z);
    orientation->set_w(quaternion.w);
    SetVector3(geometry.box.size, cube->mutable_size());
    const std::array<double, 3> color =
        ColorForLabel(observation.detection.label);
    cube->mutable_color()->set_r(color[0]);
    cube->mutable_color()->set_g(color[1]);
    cube->mutable_color()->set_b(color[2]);
    cube->mutable_color()->set_a(0.35);
    ++object_index;
  }

  std::string payload;
  if (!update.SerializeToString(&payload)) {
    throw std::runtime_error("failed to serialize Foxglove SceneUpdate");
  }
  return payload;
}

std::string SerializeFoxgloveImageAnnotations(
    std::int64_t timestamp,
    const std::vector<ObjectObservation>& observations) {
  foxglove::ImageAnnotations annotations;
  SetTimestamp(timestamp, annotations.mutable_timestamp());

  for (const ObjectObservation& observation : observations) {
    if (!observation.detection.box.Valid()) {
      continue;
    }
    const auto& box = observation.detection.box;
    const std::array<double, 3> color =
        ColorForLabel(observation.detection.label);

    auto* points = annotations.add_points();
    SetTimestamp(timestamp, points->mutable_timestamp());
    points->set_type(foxglove::PointsAnnotation_Type_LINE_LOOP);
    points->set_thickness(2.0);
    SetColor(color, 1.0, points->mutable_outline_color());
    const std::array<std::array<double, 2>, 4> corners{{
        {{static_cast<double>(box.x), static_cast<double>(box.y)}},
        {{static_cast<double>(box.x + box.width), static_cast<double>(box.y)}},
        {{static_cast<double>(box.x + box.width),
          static_cast<double>(box.y + box.height)}},
        {{static_cast<double>(box.x), static_cast<double>(box.y + box.height)}},
    }};
    for (const auto& corner : corners) {
      auto* point = points->add_points();
      point->set_x(corner[0]);
      point->set_y(corner[1]);
    }

    auto* text = annotations.add_texts();
    SetTimestamp(timestamp, text->mutable_timestamp());
    text->mutable_position()->set_x(static_cast<double>(box.x) + 4.0);
    text->mutable_position()->set_y(static_cast<double>(box.y) + 16.0);
    text->set_text(observation.detection.label + " " +
                   ConfidenceText(observation.detection.confidence));
    text->set_font_size(14.0);
    SetColor({1.0, 1.0, 1.0}, 1.0, text->mutable_text_color());
    SetColor({0.0, 0.0, 0.0}, 0.55, text->mutable_background_color());
  }

  std::string payload;
  if (!annotations.SerializeToString(&payload)) {
    throw std::runtime_error("failed to serialize Foxglove ImageAnnotations");
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
