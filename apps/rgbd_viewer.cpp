#include <chrono>
#include <cstddef>
#include <exception>
#include <fstream>
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
#include "rgbd/core/object_geometry.hpp"
#include "rgbd/core/perception_pipeline.hpp"
#include "rgbd/io/recording.hpp"
#include "rgbd/version.hpp"

namespace {

constexpr int kDefaultFrames = 60;
constexpr int kReportEvery = 10;

struct Options {
  rgbd::CameraConfig camera;
  std::string camera_uri;
  std::string recording_path;
  std::string model_path;
  std::string labels_path;
  std::string device = "CPU";
  int frames = kDefaultFrames;
  int timeout_ms = 1000;
  int input_size = 640;
  float confidence = 0.25F;
  float iou = 0.45F;
  bool filter = true;
  int cluster_stride = 2;
  float cluster_voxel_m = 0.025F;
  int min_cluster_points = 20;
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
    if (argument == "--model") {
      if (++index >= argc) {
        throw std::invalid_argument("--model requires a path");
      }
      options.model_path = argv[index];
    } else if (argument == "--labels") {
      if (++index >= argc) {
        throw std::invalid_argument("--labels requires a path");
      }
      options.labels_path = argv[index];
    } else if (argument == "--device") {
      if (++index >= argc) {
        throw std::invalid_argument("--device requires a value");
      }
      options.device = argv[index];
    } else if (argument == "--imgsz") {
      if (++index >= argc) {
        throw std::invalid_argument("--imgsz requires a value");
      }
      options.input_size = rgbd::app::ParsePositiveInt(argv[index], "--imgsz");
    } else if (argument == "--confidence") {
      if (++index >= argc) {
        throw std::invalid_argument("--confidence requires a value");
      }
      options.confidence = rgbd::app::ParseFloat(argv[index], "--confidence");
      if (options.confidence < 0.0F || options.confidence > 1.0F) {
        throw std::invalid_argument("--confidence must be in [0, 1]");
      }
    } else if (argument == "--iou") {
      if (++index >= argc) {
        throw std::invalid_argument("--iou requires a value");
      }
      options.iou = rgbd::app::ParseFloat(argv[index], "--iou");
      if (options.iou < 0.0F || options.iou > 1.0F) {
        throw std::invalid_argument("--iou must be in [0, 1]");
      }
    } else if (argument == "--no-filter") {
      options.filter = false;
    } else if (argument == "--cluster-stride") {
      if (++index >= argc) {
        throw std::invalid_argument("--cluster-stride requires a value");
      }
      options.cluster_stride =
          rgbd::app::ParsePositiveInt(argv[index], "--cluster-stride");
    } else if (argument == "--cluster-voxel") {
      if (++index >= argc) {
        throw std::invalid_argument("--cluster-voxel requires a value");
      }
      options.cluster_voxel_m =
          rgbd::app::ParseFloat(argv[index], "--cluster-voxel");
      if (options.cluster_voxel_m <= 0.0F) {
        throw std::invalid_argument("--cluster-voxel must be positive");
      }
    } else if (argument == "--min-cluster-points") {
      if (++index >= argc) {
        throw std::invalid_argument("--min-cluster-points requires a value");
      }
      options.min_cluster_points =
          rgbd::app::ParsePositiveInt(argv[index], "--min-cluster-points");
    } else if (argument == "--frames") {
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
  if (observation.geometry) {
    const rgbd::OrientedBox& box = observation.geometry->box;
    std::cout << std::fixed << std::setprecision(4) << " center=("
              << box.center.x << ", " << box.center.y << ", " << box.center.z
              << ") size=(" << box.size.x << ", " << box.size.y << ", "
              << box.size.z << ") points=" << observation.geometry->point_count;
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
    rgbd::app::RecognizerOptions recognizer_options;
    recognizer_options.model_path = options.model_path;
    recognizer_options.labels_path = options.labels_path;
    recognizer_options.device = options.device;
    recognizer_options.input_size = options.input_size;
    recognizer_options.confidence = options.confidence;
    recognizer_options.iou = options.iou;
    recognizer_options.enable_filter = options.filter;
    const std::unique_ptr<rgbd::IVisionRecognizer> recognizer =
        rgbd::app::CreateRecognizer(recognizer_options);
    rgbd::RobustDepthFusionEstimator depth_estimator(rgbd::DepthFusionConfig{
        0.75F, 5, options.camera.depth_control.min_depth_m,
        options.camera.depth_control.max_depth_m, 3.0F});
    rgbd::ObjectGeometryConfig geometry_config;
    geometry_config.pixel_stride =
        static_cast<std::size_t>(options.cluster_stride);
    geometry_config.voxel_size_m = options.cluster_voxel_m;
    geometry_config.min_cluster_points =
        static_cast<std::size_t>(options.min_cluster_points);
    geometry_config.min_depth_m = options.camera.depth_control.min_depth_m;
    geometry_config.max_depth_m = options.camera.depth_control.max_depth_m;
    rgbd::DepthPointCloudGeometryEstimator geometry_estimator(geometry_config);
    rgbd::PerceptionPipeline pipeline(*recognizer, depth_estimator,
                                      &geometry_estimator);
    std::unique_ptr<rgbd::RecordingWriter> recorder;
    if (!options.recording_path.empty()) {
      recorder =
          std::make_unique<rgbd::RecordingWriter>(options.recording_path);
    }

    rgbd::LogInfo("rgbd_perception " + std::string(rgbd::kVersion));
    if (!options.model_path.empty()) {
      rgbd::LogInfo("model=" + options.model_path +
                    " device=" + options.device);
    }
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
