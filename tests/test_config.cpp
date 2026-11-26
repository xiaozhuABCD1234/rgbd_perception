#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "rgbd/core/config.hpp"
#include "test_support.hpp"

namespace {

class TemporaryConfig {
 public:
  explicit TemporaryConfig(const std::string& content) {
    path_ = std::filesystem::temp_directory_path() /
            ("rgbd_config_test_" + std::to_string(GetProcessId()) + ".yaml");
    std::ofstream stream(path_);
    stream << content;
  }

  ~TemporaryConfig() { std::remove(path_.c_str()); }

  [[nodiscard]] std::string Path() const { return path_.string(); }

 private:
  static int GetProcessId() {
#ifdef _WIN32
    return 0;
#else
    return static_cast<int>(getpid());
#endif
  }

  std::filesystem::path path_;
};

}  // namespace

RGBD_TEST(config_reads_nested_values) {
  const TemporaryConfig file(R"(
camera:
  uri: "synthetic://?width=64&height=48"
  timeout_ms: 250
algorithm:
  depth:
    roi_scale: 0.5
    min_valid_pixels: 7
    enabled: true
)");
  const rgbd::Config config(file.Path());
  const std::string uri = config.GetString("camera.uri");
  RGBD_CHECK(uri == "synthetic://?width=64&height=48");
  RGBD_CHECK(config.GetInt("camera.timeout_ms", 0) == 250);
  RGBD_CHECK_NEAR(config.GetFloat("algorithm.depth.roi_scale", 0.0F), 0.5F,
                  1e-6F);
  RGBD_CHECK(config.GetInt("algorithm.depth.min_valid_pixels", 0) == 7);
  RGBD_CHECK(config.GetBool("algorithm.depth.enabled", false));
  RGBD_CHECK(config.Contains("camera.uri"));
  RGBD_CHECK(!config.Contains("camera.serial"));
}

RGBD_TEST(config_uses_fallbacks) {
  const TemporaryConfig file("camera:\n  uri: synthetic://\n");
  const rgbd::Config config(file.Path());
  RGBD_CHECK(config.GetString("camera.serial", "default") == "default");
  RGBD_CHECK(config.GetInt("camera.timeout_ms", 1000) == 1000);
  RGBD_CHECK(config.GetBool("camera.loop", true));
}

RGBD_TEST(config_rejects_missing_file) {
  RGBD_CHECK_THROW(rgbd::Config("/definitely/missing/rgbd.yaml"),
                   std::runtime_error);
}
