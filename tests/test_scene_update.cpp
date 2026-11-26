#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "foxglove/SceneUpdate.pb.h"
#include "foxglove_messages.hpp"
#include "rgbd/core/observation.hpp"
#include "test_support.hpp"

namespace {

rgbd::ObjectObservation MakeObservation() {
  rgbd::ObjectObservation observation;
  observation.camera_id = "scene";
  observation.sequence = 5;
  observation.timestamp = std::chrono::nanoseconds(100);
  observation.detection.label = "cup";
  observation.detection.confidence = 0.75F;
  rgbd::ObjectGeometry geometry;
  geometry.point_count = 123;
  geometry.box.center = {0.0F, 0.0F, 0.5F};
  geometry.box.size = {0.2F, 0.3F, 0.4F};
  observation.geometry = geometry;
  return observation;
}

}

RGBD_TEST(scene_update_serializes_cube) {
  const std::vector<rgbd::ObjectObservation> observations{MakeObservation()};
  const std::string payload = rgbd::internal::SerializeFoxgloveSceneUpdate(
      1000000000, "camera", observations);
  foxglove::SceneUpdate update;
  RGBD_CHECK(update.ParseFromString(payload));
  RGBD_CHECK(update.deletions_size() == 1);
  RGBD_CHECK(update.entities_size() == 1);
  const foxglove::SceneEntity& entity = update.entities(0);
  RGBD_CHECK(entity.id() == "object_0");
  RGBD_CHECK(entity.frame_id() == "camera");
  RGBD_CHECK(entity.cubes_size() == 1);
  const foxglove::CubePrimitive& cube = entity.cubes(0);
  RGBD_CHECK_NEAR(static_cast<float>(cube.pose().position().x()), 0.5F, 1e-6F);
  RGBD_CHECK_NEAR(static_cast<float>(cube.pose().position().y()), 0.0F, 1e-6F);
  RGBD_CHECK_NEAR(static_cast<float>(cube.pose().position().z()), 0.0F, 1e-6F);
  RGBD_CHECK_NEAR(static_cast<float>(cube.size().x()), 0.2F, 1e-6F);
  RGBD_CHECK_NEAR(static_cast<float>(cube.size().y()), 0.3F, 1e-6F);
  RGBD_CHECK_NEAR(static_cast<float>(cube.size().z()), 0.4F, 1e-6F);
}

RGBD_TEST(scene_update_deletes_without_objects) {
  const std::string payload =
      rgbd::internal::SerializeFoxgloveSceneUpdate(1, "camera", {});
  foxglove::SceneUpdate update;
  RGBD_CHECK(update.ParseFromString(payload));
  RGBD_CHECK(update.deletions_size() == 1);
  RGBD_CHECK(update.deletions(0).type() ==
             foxglove::SceneEntityDeletion_Type_ALL);
  RGBD_CHECK(update.entities_size() == 0);
}
