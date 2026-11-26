#ifndef RGBD_CORE_CONFIG_HPP_
#define RGBD_CORE_CONFIG_HPP_

#include <memory>
#include <string>

namespace rgbd {

class Config {
 public:
  Config();
  explicit Config(const std::string& path);
  ~Config();

  Config(const Config&) = delete;
  Config& operator=(const Config&) = delete;

  void Load(const std::string& path);

  [[nodiscard]] bool Contains(const std::string& path) const;
  [[nodiscard]] std::string GetString(const std::string& path,
                                      const std::string& fallback = "") const;
  [[nodiscard]] int GetInt(const std::string& path, int fallback) const;
  [[nodiscard]] float GetFloat(const std::string& path, float fallback) const;
  [[nodiscard]] bool GetBool(const std::string& path, bool fallback) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rgbd

#endif
