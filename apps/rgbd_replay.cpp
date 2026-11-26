#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "app_support.hpp"
#include "rgbd/core/depth_estimator.hpp"
#include "rgbd/core/logging.hpp"
#include "rgbd/core/perception_pipeline.hpp"
#include "rgbd/io/replay_camera.hpp"
#include "rgbd/recognition/center_box_recognizer.hpp"
#include "rgbd/version.hpp"

namespace {

struct Options {
  std::string path;
  int frames = 0;
  bool loop = false;
  int timeout_ms = 1000;
};

Options ParseOptions(int argc, char** argv, Options options) {
  bool path_set = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--") {
      continue;
    }
    if (argument == "--frames") {
      if (++index >= argc) {
        throw std::invalid_argument("--frames requires a value");
      }
      options.frames = rgbd::app::ParsePositiveInt(argv[index], "--frames");
    } else if (argument == "--loop") {
      options.loop = true;
    } else if (argument.rfind("--", 0) == 0) {
      throw std::invalid_argument("unknown argument: " + argument);
    } else {
      if (path_set) {
        throw std::invalid_argument("recording path specified more than once");
      }
      options.path = argument;
      path_set = true;
    }
  }
  if (options.path.empty()) {
    throw std::invalid_argument("recording path is required");
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
    rgbd::InitializeLogging(rgbd::LoggingConfig{});
    const Options options = ParseOptions(argc, argv, Options{});

    rgbd::ReplayCameraConfig camera_config;
    camera_config.path = options.path;
    camera_config.loop = options.loop;
    rgbd::ReplayCamera camera(camera_config);
    rgbd::CenterBoxRecognizer recognizer(
        rgbd::CenterBoxRecognizerConfig{"object", 0.5F, 1.0F});
    rgbd::RobustDepthFusionEstimator depth_estimator(
        rgbd::DepthFusionConfig{0.75F, 5, 0.04F, 10.0F, 3.0F});
    rgbd::PerceptionPipeline pipeline(recognizer, depth_estimator);

    rgbd::LogInfo("rgbd_perception " + std::string(rgbd::kVersion));
    camera.Open();
    camera.Start();
    int received = 0;
    while (options.frames == 0 || received < options.frames) {
      rgbd::FrameSet frame;
      if (!camera.Read(frame, std::chrono::milliseconds(options.timeout_ms))) {
        break;
      }
      ++received;
      for (const auto& observation : pipeline.Process(frame)) {
        PrintObservation(observation);
      }
    }
    camera.Stop();
    camera.Close();
    rgbd::LogInfo("replayed " + std::to_string(received) + " frames");
    rgbd::ShutdownLogging();
    return received > 0 ? 0 : 1;
  } catch (const std::exception& error) {
    rgbd::LogError(error.what());
    rgbd::ShutdownLogging();
    return 1;
  }
}
