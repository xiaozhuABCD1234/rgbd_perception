#include <chrono>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rgbd/cameras/factory.hpp"
#include "rgbd/core/depth_estimator.hpp"
#include "rgbd/core/perception_pipeline.hpp"
#include "rgbd/io/recording.hpp"
#include "rgbd/recognition/center_box_recognizer.hpp"
#include "rgbd/version.hpp"

namespace {

constexpr int kDefaultFrames = 60;
constexpr int kTimeoutMs = 1000;
constexpr int kReportEvery = 10;

struct Options {
  std::string camera_uri;
  std::string recording_path;
  int frames = kDefaultFrames;
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
  if (argc < 2) {
    throw std::invalid_argument(
        "usage: rgbd_viewer <camera-uri> [--frames N] [--record PATH]");
  }
  Options options;
  options.camera_uri = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--") {
      continue;
    }
    if (argument == "--frames") {
      if (++index >= argc) {
        throw std::invalid_argument("--frames requires a value");
      }
      options.frames = ParsePositiveInt(argv[index], "--frames");
    } else if (argument == "--record") {
      if (++index >= argc) {
        throw std::invalid_argument("--record requires a path");
      }
      options.recording_path = argv[index];
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
    rgbd::CenterBoxRecognizer recognizer;
    rgbd::RobustDepthFusionEstimator depth_estimator;
    rgbd::PerceptionPipeline pipeline(recognizer, depth_estimator);
    std::unique_ptr<rgbd::RecordingWriter> recorder;
    if (!options.recording_path.empty()) {
      recorder =
          std::make_unique<rgbd::RecordingWriter>(options.recording_path);
    }

    std::cout << "rgbd_perception " << rgbd::kVersion << '\n';
    camera->Open();
    camera->Start();
    std::cout << "camera=" << camera->Name() << '\n';
    int received = 0;
    for (int index = 0; index < options.frames; ++index) {
      rgbd::FrameSet frame;
      if (!camera->Read(frame, std::chrono::milliseconds(kTimeoutMs))) {
        std::cerr << "frame timeout\n";
        continue;
      }
      ++received;
      if (recorder != nullptr) {
        recorder->Write(frame);
      }
      if (received % kReportEvery != 0) {
        continue;
      }
      const std::size_t center =
          (frame.depth.height / 2) * frame.depth.width + frame.depth.width / 2;
      std::cout << std::fixed << std::setprecision(3) << "seq "
                << frame.sequence << " center_m " << frame.depth.values[center]
                << " fx " << frame.color_intrinsics.fx << " fy "
                << frame.color_intrinsics.fy << '\n';
      for (const auto& observation : pipeline.Process(frame)) {
        std::cout << "  label=" << observation.detection.label
                  << " quality=" << rgbd::ToString(observation.depth.quality);
        if (observation.depth.distance_m) {
          std::cout << " depth_m=" << *observation.depth.distance_m;
        }
        if (observation.depth.position_camera_m) {
          const rgbd::Point3f& position = *observation.depth.position_camera_m;
          std::cout << std::fixed << std::setprecision(4) << " xyz=("
                    << position.x << ", " << position.y << ", " << position.z
                    << ")";
        }
        std::cout << '\n';
      }
    }
    camera->Stop();
    camera->Close();
    if (recorder != nullptr) {
      recorder->Close();
      std::cout << "recorded=" << recorder->FrameCount()
                << " path=" << recorder->Path() << '\n';
    }
    std::cout << "received " << received << " of " << options.frames << '\n';
    return received > 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
