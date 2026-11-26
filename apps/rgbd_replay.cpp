#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rgbd/core/depth_estimator.hpp"
#include "rgbd/core/perception_pipeline.hpp"
#include "rgbd/io/replay_camera.hpp"
#include "rgbd/recognition/center_box_recognizer.hpp"
#include "rgbd/version.hpp"

namespace {

constexpr int kTimeoutMs = 1000;

struct Options {
  std::string path;
  int frames = 0;
  bool loop = false;
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
        "usage: rgbd_replay <recording> [--frames N] [--loop]");
  }
  Options options;
  options.path = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--frames") {
      if (++index >= argc) {
        throw std::invalid_argument("--frames requires a value");
      }
      options.frames = ParsePositiveInt(argv[index], "--frames");
    } else if (argument == "--loop") {
      options.loop = true;
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
    rgbd::ReplayCameraConfig camera_config;
    camera_config.path = options.path;
    camera_config.loop = options.loop;
    rgbd::ReplayCamera camera(camera_config);
    rgbd::CenterBoxRecognizer recognizer;
    rgbd::RobustDepthFusionEstimator depth_estimator;
    rgbd::PerceptionPipeline pipeline(recognizer, depth_estimator);

    std::cout << "rgbd_perception " << rgbd::kVersion << '\n';
    camera.Open();
    camera.Start();
    int received = 0;
    while (options.frames == 0 || received < options.frames) {
      rgbd::FrameSet frame;
      if (!camera.Read(frame, std::chrono::milliseconds(kTimeoutMs))) {
        break;
      }
      ++received;
      for (const auto& observation : pipeline.Process(frame)) {
        std::cout << "frame=" << observation.sequence
                  << " label=" << observation.detection.label
                  << " quality=" << rgbd::ToString(observation.depth.quality);
        if (observation.depth.distance_m) {
          std::cout << std::fixed << std::setprecision(3)
                    << " depth_m=" << *observation.depth.distance_m;
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
    camera.Stop();
    camera.Close();
    std::cout << "replayed=" << received << '\n';
    return received > 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
