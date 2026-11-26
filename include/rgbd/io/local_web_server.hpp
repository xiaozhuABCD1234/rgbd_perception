#ifndef RGBD_IO_LOCAL_WEB_SERVER_HPP_
#define RGBD_IO_LOCAL_WEB_SERVER_HPP_

#include <memory>
#include <string>

namespace rgbd {

struct LocalWebServerConfig {
  std::string web_root = "data/lichtblick-web";
  std::string host = "127.0.0.1";
  int port = 8080;
  std::string websocket_url;
};

class LocalWebServer {
 public:
  explicit LocalWebServer(LocalWebServerConfig config);
  ~LocalWebServer();

  LocalWebServer(const LocalWebServer&) = delete;
  LocalWebServer& operator=(const LocalWebServer&) = delete;

  void Start();
  void Stop();

  [[nodiscard]] std::string Url() const;
  [[nodiscard]] std::string BrowserUrl() const;
  [[nodiscard]] int Port() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}

#endif
