#include <chrono>
#include <csignal>
#include <cstddef>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rgbd/cameras/factory.hpp"
#include "rgbd/io/mcap_image_writer.hpp"
#include "rgbd/version.hpp"

namespace {

constexpr int kTimeoutMs = 1000;
constexpr int kReportEvery = 30;

volatile std::sig_atomic_t g_stop = 0;

void HandleSignal(int) { g_stop = 1; }

struct Options {
  std::string camera_uri = "gemini305://?width=640&height=480&fps=30";
  std::string output = "rgbd_lichtblick.mcap";
  int frames = 0;
};

int ParsePositiveInt(const std::string& value, const char* name) {
  std::size_t parsed = 0;
  const int result = std::stoi(value, &parsed);
  if (parsed != value.size() || result <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return result;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--camera") {
      if (++index >= argc) {
        throw std::invalid_argument("--camera requires a value");
      }
      options.camera_uri = argv[index];
    } else if (argument == "--output") {
      if (++index >= argc) {
        throw std::invalid_argument("--output requires a path");
      }
      options.output = argv[index];
    } else if (argument == "--frames") {
      if (++index >= argc) {
        throw std::invalid_argument("--frames requires a value");
      }
      options.frames = ParsePositiveInt(argv[index], "--frames");
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  return options;
}

}

int main(int argc, char** argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    auto camera = rgbd::CreateCameraFromUri(options.camera_uri);
    rgbd::McapImageWriterConfig writer_config;
    writer_config.path = options.output;
    rgbd::McapImageWriter writer(writer_config);

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << "rgbd_perception " << rgbd::kVersion << '\n';
    std::cout << "recording " << options.output << '\n';
    std::cout << "open the file in Lichtblick after the capture stops\n";
    camera->Open();
    camera->Start();
    std::cout << "camera=" << camera->Name() << '\n';

    int received = 0;
    while (!g_stop && (options.frames == 0 || received < options.frames)) {
      rgbd::FrameSet frame;
      if (!camera->Read(frame, std::chrono::milliseconds(kTimeoutMs))) {
        if (g_stop) {
          break;
        }
        throw std::runtime_error("failed to read camera frame");
      }
      writer.Write(frame);
      ++received;
      if (received % kReportEvery == 0) {
        std::cout << "frames=" << received << " sequence=" << frame.sequence
                  << '\n';
      }
    }

    camera->Stop();
    camera->Close();
    writer.Close();
    std::cout << "saved=" << options.output << " frames=" << writer.FrameCount()
              << '\n';
    return received > 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
