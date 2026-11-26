#include "rgbd/core/config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace rgbd {

namespace {

std::runtime_error InvalidValue(const std::string& path) {
  return std::runtime_error("invalid config value: " + path);
}

}  // namespace

class Config::Impl {
 public:
  YAML::Node root{YAML::NodeType::Map};

  [[nodiscard]] YAML::Node Find(const std::string& path) const {
    YAML::Node node = YAML::Clone(root);
    std::size_t begin = 0;
    while (begin <= path.size()) {
      const std::size_t end = path.find('.', begin);
      const std::string key = path.substr(
          begin, end == std::string::npos ? std::string::npos : end - begin);
      if (!node.IsMap()) {
        return {};
      }
      const YAML::Node current = node;
      const YAML::Node child = current[key];
      if (!child || child.IsNull()) {
        return {};
      }
      node = YAML::Clone(child);
      if (end == std::string::npos) {
        break;
      }
      begin = end + 1;
    }
    return node;
  }
};

Config::Config() : impl_(std::make_unique<Impl>()) {}

Config::Config(const std::string& path) : Config() { Load(path); }

Config::~Config() = default;

void Config::Load(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("failed to load config " + path + ": " +
                             error.what());
  }
  if (root.IsNull()) {
    root = YAML::Node(YAML::NodeType::Map);
  }
  if (!root.IsMap()) {
    throw std::runtime_error("config root must be a mapping: " + path);
  }
  impl_->root = root;
}

bool Config::Contains(const std::string& path) const {
  const YAML::Node node = impl_->Find(path);
  return node.IsDefined() && !node.IsNull();
}

std::string Config::GetString(const std::string& path,
                              const std::string& fallback) const {
  const YAML::Node node = impl_->Find(path);
  if (!node || node.IsNull()) {
    return fallback;
  }
  if (!node.IsScalar()) {
    throw InvalidValue(path);
  }
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception&) {
    throw InvalidValue(path);
  }
}

int Config::GetInt(const std::string& path, int fallback) const {
  const YAML::Node node = impl_->Find(path);
  if (!node || node.IsNull()) {
    return fallback;
  }
  if (!node.IsScalar()) {
    throw InvalidValue(path);
  }
  try {
    return node.as<int>();
  } catch (const YAML::Exception&) {
    throw InvalidValue(path);
  }
}

float Config::GetFloat(const std::string& path, float fallback) const {
  const YAML::Node node = impl_->Find(path);
  if (!node || node.IsNull()) {
    return fallback;
  }
  if (!node.IsScalar()) {
    throw InvalidValue(path);
  }
  try {
    return node.as<float>();
  } catch (const YAML::Exception&) {
    throw InvalidValue(path);
  }
}

bool Config::GetBool(const std::string& path, bool fallback) const {
  const YAML::Node node = impl_->Find(path);
  if (!node || node.IsNull()) {
    return fallback;
  }
  if (!node.IsScalar()) {
    throw InvalidValue(path);
  }
  try {
    return node.as<bool>();
  } catch (const YAML::Exception&) {
    throw InvalidValue(path);
  }
}

}  // namespace rgbd
