#include <chrono>
#include <csignal>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include "app_support.hpp"
#include "rgbd/cameras/factory.hpp"
#include "rgbd/core/config.hpp"
#include "rgbd/core/logging.hpp"
#include "rgbd/io/lichtblick_server.hpp"
#include "rgbd/io/local_web_server.hpp"
#include "rgbd/io/mcap_image_writer.hpp"
#include "rgbd/version.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void HandleSignal(int) { g_stop = 1; }

struct Options {
  rgbd::CameraConfig camera;
  std::string camera_uri;
  std::string output;
  int frames = 0;
  int port = 8765;
  int http_port = 8080;
  std::string web_root = "data/lichtblick-web";
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
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--") {
      continue;
    }
    if (argument == "--config") {
      ++index;
      continue;
    }
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
      options.frames = rgbd::app::ParsePositiveInt(argv[index], "--frames");
    } else if (argument == "--port") {
      if (++index >= argc) {
        throw std::invalid_argument("--port requires a value");
      }
      options.port = rgbd::app::ParsePositiveInt(argv[index], "--port");
    } else if (argument == "--http-port") {
      if (++index >= argc) {
        throw std::invalid_argument("--http-port requires a value");
      }
      options.http_port =
          rgbd::app::ParsePositiveInt(argv[index], "--http-port");
    } else if (argument == "--web-root") {
      if (++index >= argc) {
        throw std::invalid_argument("--web-root requires a path");
      }
      options.web_root = argv[index];
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  return options;
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
    rgbd::LichtblickServerConfig server_config;
    server_config.port = options.port;
    server_config.min_depth_m = options.camera.depth_control.min_depth_m;
    server_config.max_depth_m = options.camera.depth_control.max_depth_m;
    rgbd::LichtblickServer server(server_config);
    rgbd::LocalWebServerConfig web_config;
    web_config.web_root = options.web_root;
    web_config.port = options.http_port;
    web_config.websocket_url = server.Url();
    rgbd::LocalWebServer web_server(web_config);
    std::unique_ptr<rgbd::McapImageWriter> writer;
    if (!options.output.empty()) {
      rgbd::McapImageWriterConfig writer_config;
      writer_config.path = options.output;
      writer_config.min_depth_m = options.camera.depth_control.min_depth_m;
      writer_config.max_depth_m = options.camera.depth_control.max_depth_m;
      writer = std::make_unique<rgbd::McapImageWriter>(writer_config);
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    rgbd::LogInfo("rgbd_perception " + std::string(rgbd::kVersion));
    rgbd::LogInfo("WebSocket: " + server.Url());
    if (writer != nullptr) {
      rgbd::LogInfo("recording: " + options.output);
    }
    server.Start();
    web_server.Start();
    rgbd::LogInfo("Browser: " + web_server.BrowserUrl());
    camera->Open();
    camera->Start();
    if (options.camera_uri.empty()) {
      rgbd::LogInfo("camera=" + camera->Name() + " " +
                    rgbd::app::DescribeCamera(options.camera));
    } else {
      rgbd::LogInfo("camera=" + camera->Name());
    }

    int received = 0;
    while (!g_stop && (options.frames == 0 || received < options.frames)) {
      rgbd::FrameSet frame;
      if (!camera->Read(frame, std::chrono::milliseconds(options.timeout_ms))) {
        if (g_stop) {
          break;
        }
        throw std::runtime_error("failed to read camera frame");
      }
      server.Publish(frame);
      if (writer != nullptr) {
        writer->Write(frame);
      }
      ++received;
    }

    camera->Stop();
    camera->Close();
    web_server.Stop();
    server.Stop();
    if (writer != nullptr) {
      writer->Close();
      rgbd::LogInfo("saved " + options.output + " (" +
                    std::to_string(writer->FrameCount()) + " frames)");
    }
    rgbd::ShutdownLogging();
    return received > 0 ? 0 : 1;
  } catch (const std::exception& error) {
    rgbd::LogError(error.what());
    rgbd::ShutdownLogging();
    return 1;
  }
}
