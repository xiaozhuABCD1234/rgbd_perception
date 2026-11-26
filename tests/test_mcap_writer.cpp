#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include "rgbd/io/mcap_image_writer.hpp"
#include "test_support.hpp"

namespace {

rgbd::FrameSet MakeFrame(std::uint64_t sequence) {
  constexpr std::size_t width = 8;
  constexpr std::size_t height = 6;
  rgbd::FrameSet frame;
  frame.camera_id = "mcap";
  frame.sequence = sequence;
  frame.color.width = width;
  frame.color.height = height;
  frame.color.pixels.assign(width * height * rgbd::ColorImage::channels, 42);
  frame.depth.width = width;
  frame.depth.height = height;
  frame.depth.values.assign(width * height, 2.0F);
  frame.color_intrinsics = {width, height, 10.0F, 10.0F, 3.5F, 2.5F};
  return frame;
}

RGBD_TEST(mcap_writer_creates_file) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "rgbd_perception_writer.mcap";
  std::filesystem::remove(path);
  {
    rgbd::McapImageWriterConfig config;
    config.path = path.string();
    rgbd::McapImageWriter writer(config);
    writer.Write(MakeFrame(1));
    writer.Write(MakeFrame(2));
    RGBD_CHECK(writer.FrameCount() == 2);
    writer.Close();
  }
  RGBD_CHECK(std::filesystem::exists(path));
  std::ifstream stream(path, std::ios::binary);
  std::array<char, 8> magic{};
  stream.read(magic.data(), magic.size());
  const std::array<char, 8> expected{
      {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'}};
  RGBD_CHECK(magic == expected);
  stream.close();
  std::filesystem::remove(path);
}

RGBD_TEST(mcap_writer_validates_input) {
  rgbd::McapImageWriterConfig config;
  RGBD_CHECK_THROW(rgbd::McapImageWriter writer(config), std::invalid_argument);
  config.path = (std::filesystem::temp_directory_path() /
                 "rgbd_perception_writer_invalid.mcap")
                    .string();
  rgbd::McapImageWriter writer(config);
  RGBD_CHECK_THROW(writer.Write(rgbd::FrameSet{}), std::invalid_argument);
  writer.Close();
  RGBD_CHECK_THROW(writer.Write(MakeFrame(1)), std::logic_error);
  std::filesystem::remove(config.path);
}

}
