#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "rgbd/io/recording.hpp"
#include "rgbd/io/replay_camera.hpp"
#include "test_support.hpp"

namespace {

rgbd::FrameSet MakeFrame(std::uint64_t sequence, float depth) {
  constexpr std::size_t width = 8;
  constexpr std::size_t height = 6;
  rgbd::FrameSet frame;
  frame.camera_id = "recording";
  frame.sequence = sequence;
  frame.device_timestamp = std::chrono::nanoseconds(sequence * 1000);
  frame.host_timestamp = std::chrono::nanoseconds(sequence * 2000);
  frame.color.width = width;
  frame.color.height = height;
  frame.color.pixels.resize(width * height * rgbd::ColorImage::channels);
  for (std::size_t index = 0; index < frame.color.pixels.size(); ++index) {
    frame.color.pixels[index] = static_cast<std::uint8_t>(index);
  }
  frame.depth.width = width;
  frame.depth.height = height;
  frame.depth.values.assign(width * height, depth);
  frame.color_intrinsics = {width, height, 10.0F, 11.0F, 3.5F, 2.5F};
  return frame;
}

std::filesystem::path TempPath(const std::string& name) {
  return std::filesystem::temp_directory_path() / name;
}

RGBD_TEST(recording_roundtrip) {
  const std::filesystem::path path = TempPath("rgbd_perception_roundtrip.rgbd");
  std::filesystem::remove(path);
  const rgbd::FrameSet first = MakeFrame(3, 1.25F);
  const rgbd::FrameSet second = MakeFrame(4, 2.5F);
  {
    rgbd::RecordingWriter writer(path.string());
    writer.Write(first);
    writer.Write(second);
    RGBD_CHECK(writer.FrameCount() == 2);
    writer.Close();
  }
  RGBD_CHECK(std::filesystem::exists(path));

  rgbd::ReplayCameraConfig config;
  config.path = path.string();
  rgbd::ReplayCamera camera(config);
  camera.Open();
  camera.Start();
  rgbd::FrameSet loaded;
  RGBD_CHECK(camera.Read(loaded, std::chrono::milliseconds(1)));
  RGBD_CHECK(loaded.camera_id == first.camera_id);
  RGBD_CHECK(loaded.sequence == first.sequence);
  RGBD_CHECK(loaded.device_timestamp == first.device_timestamp);
  RGBD_CHECK(loaded.color.width == first.color.width);
  RGBD_CHECK(loaded.depth.values.front() == first.depth.values.front());
  RGBD_CHECK(loaded.color.pixels == first.color.pixels);
  RGBD_CHECK(camera.Read(loaded, std::chrono::milliseconds(1)));
  RGBD_CHECK(loaded.sequence == second.sequence);
  RGBD_CHECK(!camera.Read(loaded, std::chrono::milliseconds(1)));
  camera.Stop();
  RGBD_CHECK(!camera.Read(loaded, std::chrono::milliseconds(1)));
  camera.Close();
  std::filesystem::remove(path);
}

RGBD_TEST(recording_loop_replays_first_frame) {
  const std::filesystem::path path = TempPath("rgbd_perception_loop.rgbd");
  std::filesystem::remove(path);
  const rgbd::FrameSet frame = MakeFrame(9, 1.0F);
  {
    rgbd::RecordingWriter writer(path.string());
    writer.Write(frame);
    writer.Close();
  }
  rgbd::ReplayCameraConfig config;
  config.path = path.string();
  config.loop = true;
  rgbd::ReplayCamera camera(config);
  camera.Open();
  camera.Start();
  rgbd::FrameSet loaded;
  RGBD_CHECK(camera.Read(loaded, std::chrono::milliseconds(1)));
  RGBD_CHECK(camera.Read(loaded, std::chrono::milliseconds(1)));
  RGBD_CHECK(loaded.sequence == frame.sequence);
  camera.Close();
  std::filesystem::remove(path);
}

RGBD_TEST(recording_rejects_invalid_files) {
  const std::filesystem::path path = TempPath("rgbd_perception_invalid.rgbd");
  std::filesystem::remove(path);
  {
    std::ofstream stream(path, std::ios::binary);
    stream << "not a recording";
  }
  rgbd::ReplayCameraConfig config;
  config.path = path.string();
  rgbd::ReplayCamera camera(config);
  RGBD_CHECK_THROW(camera.Open(), std::runtime_error);
  std::filesystem::remove(path);

  config.path = path.string();
  rgbd::ReplayCamera missing(config);
  RGBD_CHECK_THROW(missing.Open(), std::runtime_error);
}

RGBD_TEST(recording_rejects_invalid_frame) {
  const std::filesystem::path path =
      TempPath("rgbd_perception_invalid_frame.rgbd");
  std::filesystem::remove(path);
  {
    rgbd::RecordingWriter writer(path.string());
    RGBD_CHECK_THROW(writer.Write(rgbd::FrameSet{}), std::invalid_argument);
    writer.Close();
    RGBD_CHECK_THROW(writer.Write(MakeFrame(1, 1.0F)), std::logic_error);
  }
  std::filesystem::remove(path);
}

}
