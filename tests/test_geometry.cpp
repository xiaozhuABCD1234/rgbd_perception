#include <cmath>

#include "rgbd/core/geometry.hpp"
#include "test_support.hpp"

namespace {

RGBD_TEST(geometry_identity_transform) {
  const rgbd::RigidTransform transform = rgbd::RigidTransform::Identity();
  const rgbd::Point3f point = transform.Apply({1.0F, 2.0F, 3.0F});
  RGBD_CHECK_NEAR(point.x, 1.0F, 1e-6F);
  RGBD_CHECK_NEAR(point.y, 2.0F, 1e-6F);
  RGBD_CHECK_NEAR(point.z, 3.0F, 1e-6F);
}

RGBD_TEST(geometry_rigid_transform) {
  rgbd::RigidTransform transform;
  transform.rotation = {0.0F, -1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  transform.translation = {1.0F, 2.0F, 3.0F};
  const rgbd::Point3f point = transform.Apply({1.0F, 2.0F, 3.0F});
  RGBD_CHECK_NEAR(point.x, -1.0F, 1e-6F);
  RGBD_CHECK_NEAR(point.y, 3.0F, 1e-6F);
  RGBD_CHECK_NEAR(point.z, 6.0F, 1e-6F);
}

RGBD_TEST(geometry_unproject) {
  const rgbd::CameraIntrinsics intrinsics{640,    480,    500.0F,
                                          500.0F, 319.5F, 239.5F};
  const auto point = rgbd::Unproject(intrinsics, 320.5F, 240.5F, 2.0F);
  RGBD_CHECK(point.has_value());
  RGBD_CHECK_NEAR(point->x, 0.004F, 1e-5F);
  RGBD_CHECK_NEAR(point->y, 0.004F, 1e-5F);
  RGBD_CHECK_NEAR(point->z, 2.0F, 1e-6F);
  RGBD_CHECK(!rgbd::Unproject(intrinsics, 320.0F, 240.0F, 0.0F).has_value());
}

}
