#include <chrono>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "app_support.hpp"
#include "rgbd/cameras/factory.hpp"
#include "rgbd/core/config.hpp"
#include "rgbd/core/depth_estimator.hpp"
#include "rgbd/core/logging.hpp"
#include "rgbd/core/perception_pipeline.hpp"
#include "rgbd/io/recording.hpp"
#include "rgbd/recognition/center_box_recognizer.hpp"
#include "rgbd/version.hpp"

namespace {

constexpr int kDefaultFrames = 60;
constexpr int kReportEvery = 10;

struct Options {
  rgbd::CameraConfig camera;
  std::string camera_uri;
  std::string recording_path;
  int frames = kDefaultFrames;
  int timeout_ms = 1000;
};

Options LoadOptions(const rgbd::Config& config) {
  Options options;
  const rgbd::app::CameraOptions camera = rgbd::app::LoadCameraOptions(config);
  options.camera = camera.camera;
  options.timeout_ms = camera.timeout_ms;
  return options;
}

Options ParseOptions(int argc, char** argv, Options options) {
  bool camera_set = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--") {
      continue;
    }
    if (argument == "--config") {
      ++index;
      continue;
    }
    if (argument == "--frames") {
      if (++index >= argc) {
        throw std::invalid_argument("--frames requires a value");
      }
      options.frames = rgbd::app::ParsePositiveInt(argv[index], "--frames");
    } else if (argument == "--record") {
      if (++index >= argc) {
        throw std::invalid_argument("--record requires a path");
      }
      options.recording_path = argv[index];
    } else if (argument.rfind("--", 0) == 0) {
      throw std::invalid_argument("unknown argument: " + argument);
    } else {
      if (camera_set) {
        throw std::invalid_argument("camera URI specified more than once");
      }
      options.camera_uri = argument;
      camera_set = true;
    }
  }
  return options;
}

void PrintObservation(const rgbd::ObjectObservation& observation) {
  std::cout << "frame=" << observation.sequence
            << " label=" << observation.detection.label
            << " quality=" << rgbd::ToString(observation.depth.quality);
  if (observation.depth.distance_m) {
    std::cout << std::fixed << std::setprecision(3)
              << " depth_m=" << *observation.depth.distance_m;
  }
  if (observation.depth.position_camera_m) {
    const rgbd::Point3f& position = *observation.depth.position_camera_m;
    std::cout << std::fixed << std::setprecision(4) << " xyz=(" << position.x
              << ", " << position.y << ", " << position.z << ")";
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const rgbd::app::ConfigArgument config_argument =
        rgbd::app::ParseConfigArgument(argc, argv);
    rgbd::Config config;
    rgbd::app::LoadConfig(config, config_argument);
    rgbd::InitializeLogging(rgbd::LoggingConfig{});
    const Options options = ParseOptions(argc, argv, LoadOptions(config));
    auto camera = options.camera_uri.empty()
                      ? rgbd::CreateCamera(options.camera)
                      : rgbd::CreateCameraFromUri(options.camera_uri);
    rgbd::CenterBoxRecognizer recognizer(
        rgbd::CenterBoxRecognizerConfig{"object", 0.5F, 1.0F});
    rgbd::RobustDepthFusionEstimator depth_estimator(
        rgbd::DepthFusionConfig{0.75F, 5, 0.04F, 1.0F, 3.0F});
    rgbd::PerceptionPipeline pipeline(recognizer, depth_estimator);
    std::unique_ptr<rgbd::RecordingWriter> recorder;
    if (!options.recording_path.empty()) {
      recorder =
          std::make_unique<rgbd::RecordingWriter>(options.recording_path);
    }

    rgbd::LogInfo("rgbd_perception " + std::string(rgbd::kVersion));
    camera->Open();
    camera->Start();
    if (options.camera_uri.empty()) {
      rgbd::LogInfo("camera=" + camera->Name() + " " +
                    rgbd::app::DescribeCamera(options.camera));
    } else {
      rgbd::LogInfo("camera=" + camera->Name());
    }
    int received = 0;
    for (int index = 0; index < options.frames; ++index) {
      rgbd::FrameSet frame;
      if (!camera->Read(frame, std::chrono::milliseconds(options.timeout_ms))) {
        rgbd::LogWarn("frame timeout");
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
        PrintObservation(observation);
      }
    }
    camera->Stop();
    camera->Close();
    if (recorder != nullptr) {
      recorder->Close();
      rgbd::LogInfo("recorded " + recorder->Path() + " (" +
                    std::to_string(recorder->FrameCount()) + " frames)");
    }
    rgbd::LogInfo("received " + std::to_string(received) + " of " +
                  std::to_string(options.frames));
    rgbd::ShutdownLogging();
    return received > 0 ? 0 : 1;
  } catch (const std::exception& error) {
    rgbd::LogError(error.what());
    rgbd::ShutdownLogging();
    return 1;
  }
}
