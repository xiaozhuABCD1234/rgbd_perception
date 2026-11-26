#ifndef RGBD_CORE_LOGGING_HPP_
#define RGBD_CORE_LOGGING_HPP_

#include <cstddef>
#include <string>

namespace rgbd {

class Config;

struct LoggingConfig {
  std::string level = "info";
  std::string pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] %v";
  std::string file;
  std::size_t queue_size = 8192;
  std::size_t threads = 1;
};

[[nodiscard]] LoggingConfig LoadLoggingConfig(const Config& config);
void InitializeLogging(const LoggingConfig& config);
void ShutdownLogging();

void LogInfo(const std::string& message);
void LogWarn(const std::string& message);
void LogError(const std::string& message);

}  // namespace rgbd

#endif
