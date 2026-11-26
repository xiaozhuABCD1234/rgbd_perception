#include "rgbd/core/logging.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

std::atomic<bool> g_logging_initialized{false};

}  // namespace

void InitializeLogging(const LoggingConfig& config) {
  if (g_logging_initialized.exchange(true)) {
    return;
  }
  spdlog::level::level_enum level = spdlog::level::from_str(config.level);
  if (level == spdlog::level::off && config.level != "off") {
    g_logging_initialized = false;
    throw std::invalid_argument("invalid logging level: " + config.level);
  }
  try {
    spdlog::init_thread_pool(config.queue_size, config.threads);
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
    if (!config.file.empty()) {
      sinks.push_back(
          std::make_shared<spdlog::sinks::basic_file_sink_mt>(config.file));
    }
    auto logger = std::make_shared<spdlog::async_logger>(
        "rgbd", sinks.begin(), sinks.end(), spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
    logger->set_pattern(config.pattern);
    logger->set_level(level);
    logger->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(std::move(logger));
  } catch (...) {
    g_logging_initialized = false;
    throw;
  }
}

void ShutdownLogging() {
  if (!g_logging_initialized.exchange(false)) {
    return;
  }
  spdlog::shutdown();
}

void LogInfo(const std::string& message) { spdlog::info("{}", message); }

void LogWarn(const std::string& message) { spdlog::warn("{}", message); }

void LogError(const std::string& message) { spdlog::error("{}", message); }

}  // namespace rgbd
