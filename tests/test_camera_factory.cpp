#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rgbd/cameras/factory.hpp"
#include "rgbd/io/recording.hpp"
#include "test_support.hpp"

namespace {

RGBD_TEST(camera_uri_parses_parameters) {
  const auto uri = rgbd::ParseCameraUri(
      "synthetic://?width=32&height=24&depth_m=1.5&fps=10&id=test%20cam");
  RGBD_CHECK(uri.has_value());
  RGBD_CHECK(uri->scheme == "synthetic");
  RGBD_CHECK(uri->source.empty());
  RGBD_CHECK(uri->parameters.at("width") == "32");
  RGBD_CHECK(uri->parameters.at("id") == "test cam");
  RGBD_CHECK(
      rgbd::ParseCameraUri("replay:///tmp/a.rgbd?loop=true").has_value());
  RGBD_CHECK(rgbd::ParseCameraUri("replay:///tmp/a.rgbd?loop=true")->source ==
             "/tmp/a.rgbd");
}

RGBD_TEST(camera_uri_rejects_invalid_input) {
  RGBD_CHECK(!rgbd::ParseCameraUri("synthetic").has_value());
  RGBD_CHECK(!rgbd::ParseCameraUri("://missing-scheme").has_value());
  RGBD_CHECK(!rgbd::ParseCameraUri("synthetic://?bad").has_value());
  RGBD_CHECK(!rgbd::ParseCameraUri("synthetic://?x=%ZZ").has_value());
  RGBD_CHECK(!rgbd::ParseCameraUri("bad scheme://x").has_value());
}

RGBD_TEST(camera_factory_creates_synthetic_camera) {
  auto camera = rgbd::CreateCameraFromUri(
      "synthetic://?width=32&height=24&depth_m=1.5&fps=10");
  RGBD_CHECK(camera != nullptr);
  RGBD_CHECK(camera->Name() == "synthetic");
  camera->Open();
  camera->Start();
  rgbd::FrameSet frame;
  RGBD_CHECK(camera->Read(frame, std::chrono::milliseconds(1)));
  RGBD_CHECK(frame.color.width == 32);
  RGBD_CHECK(frame.color.height == 24);
  RGBD_CHECK_NEAR(frame.depth.values.front(), 1.5F, 1e-6F);
  camera->Close();
}

RGBD_TEST(camera_factory_lists_available_schemes) {
  const std::vector<std::string> schemes = rgbd::AvailableCameraSchemes();
  bool has_synthetic = false;
  bool has_replay = false;
  for (const std::string& scheme : schemes) {
    has_synthetic = has_synthetic || scheme == "synthetic";
    has_replay = has_replay || scheme == "replay";
  }
  RGBD_CHECK(has_synthetic);
  RGBD_CHECK(has_replay);
}

RGBD_TEST(camera_factory_rejects_unknown_scheme) {
  RGBD_CHECK_THROW(rgbd::CreateCameraFromUri("unknown://"),
                   std::invalid_argument);
  RGBD_CHECK_THROW(rgbd::CreateCameraFromUri("not-a-uri"),
                   std::invalid_argument);
}

RGBD_TEST(camera_factory_creates_replay_camera) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "rgbd_perception_factory.rgbd";
  std::filesystem::remove(path);
  rgbd::FrameSet frame;
  frame.camera_id = "factory";
  frame.color.width = 4;
  frame.color.height = 3;
  frame.color.pixels.assign(4 * 3 * rgbd::ColorImage::channels, 0);
  frame.depth.width = 4;
  frame.depth.height = 3;
  frame.depth.values.assign(4 * 3, 1.0F);
  frame.color_intrinsics = {4, 3, 10.0F, 10.0F, 1.5F, 1.0F};
  {
    rgbd::RecordingWriter writer(path.string());
    writer.Write(frame);
    writer.Close();
  }
  auto camera =
      rgbd::CreateCameraFromUri("replay://" + path.string() + "?loop=true");
  RGBD_CHECK(camera != nullptr);
  camera->Open();
  camera->Start();
  rgbd::FrameSet loaded;
  RGBD_CHECK(camera->Read(loaded, std::chrono::milliseconds(1)));
  RGBD_CHECK(camera->Read(loaded, std::chrono::milliseconds(1)));
  RGBD_CHECK(loaded.camera_id == "factory");
  camera->Close();
  std::filesystem::remove(path);
}

}
