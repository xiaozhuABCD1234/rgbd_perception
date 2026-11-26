#ifndef RGBD_IO_LICHTBLICK_SERVER_HPP_
#define RGBD_IO_LICHTBLICK_SERVER_HPP_

#include <memory>
#include <string>

#include "rgbd/core/frame.hpp"

namespace rgbd {

struct LichtblickServerConfig {
  int port = 8765;
  std::string name = "rgbd_perception";
  std::string frame_id = "camera";
  float min_depth_m = 0.04F;
  float max_depth_m = 1.0F;
};

class LichtblickServer {
 public:
  explicit LichtblickServer(LichtblickServerConfig config);
  ~LichtblickServer();

  LichtblickServer(const LichtblickServer&) = delete;
  LichtblickServer& operator=(const LichtblickServer&) = delete;

  void Start();
  void Stop();
  void Publish(const FrameSet& frame);

  [[nodiscard]] std::string Url() const;
  [[nodiscard]] int Port() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}

#endif
