#include "rgbd/io/local_web_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

constexpr char kLayoutJson[] = R"({
  "configById": {
    "Image!color": {
      "imageMode": {
        "imageTopic": "/camera/color/image_raw",
        "calibrationTopic": "/camera/color/camera_info",
        "synchronize": false,
        "rotation": 0,
        "annotations": {}
      }
    },
    "Image!depth": {
      "imageMode": {
        "imageTopic": "/camera/depth/image_raw",
        "calibrationTopic": "/camera/color/camera_info",
        "colorMode": "colormap",
        "colorMap": "turbo",
        "synchronize": false,
        "rotation": 0,
        "annotations": {}
      }
    }
  },
  "globalVariables": {},
  "userNodes": {},
  "playbackConfig": {
    "speed": 1
  },
  "layout": {
    "first": "Image!color",
    "second": "Image!depth",
    "direction": "row",
    "splitPercentage": 50
  }
})";

std::string UrlEncode(const std::string& input) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string output;
  for (const unsigned char character : input) {
    if ((character >= 'a' && character <= 'z') ||
        (character >= 'A' && character <= 'Z') ||
        (character >= '0' && character <= '9') || character == '-' ||
        character == '_' || character == '.' || character == '~') {
      output.push_back(static_cast<char>(character));
    } else {
      output.push_back('%');
      output.push_back(kHex[character >> 4U]);
      output.push_back(kHex[character & 0x0FU]);
    }
  }
  return output;
}

std::string UrlDecode(const std::string& input) {
  std::string output;
  for (std::size_t index = 0; index < input.size(); ++index) {
    if (input[index] == '%' && index + 2 < input.size()) {
      int value = 0;
      std::istringstream stream(input.substr(index + 1, 2));
      stream >> std::hex >> value;
      if (!stream.fail()) {
        output.push_back(static_cast<char>(value));
        index += 2;
        continue;
      }
    }
    output.push_back(input[index]);
  }
  return output;
}

std::string MimeType(const std::string& path) {
  const std::size_t dot = path.find_last_of('.');
  if (dot == std::string::npos) {
    return "application/octet-stream";
  }
  const std::string extension = path.substr(dot + 1);
  if (extension == "html") {
    return "text/html; charset=utf-8";
  }
  if (extension == "js") {
    return "application/javascript";
  }
  if (extension == "css") {
    return "text/css";
  }
  if (extension == "json" || extension == "map") {
    return "application/json";
  }
  if (extension == "png") {
    return "image/png";
  }
  if (extension == "jpg" || extension == "jpeg") {
    return "image/jpeg";
  }
  if (extension == "svg") {
    return "image/svg+xml";
  }
  if (extension == "wasm") {
    return "application/wasm";
  }
  if (extension == "woff2") {
    return "font/woff2";
  }
  if (extension == "ico") {
    return "image/x-icon";
  }
  return "application/octet-stream";
}

bool SendAll(int socket, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const char*>(data);
  std::size_t sent = 0;
  while (sent < size) {
    const ssize_t result =
        send(socket, bytes + sent, size - sent, MSG_NOSIGNAL);
    if (result <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(result);
  }
  return true;
}

bool SendResponse(int socket, const std::string& status,
                  const std::string& content_type, const std::string& body,
                  bool head_only) {
  std::ostringstream header;
  header << "HTTP/1.1 " << status << "\r\n"
         << "Content-Type: " << content_type << "\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Cache-Control: no-cache\r\n"
         << "Access-Control-Allow-Origin: *\r\n"
         << "Connection: close\r\n\r\n";
  if (!SendAll(socket, header.str().data(), header.str().size())) {
    return false;
  }
  return head_only || body.empty() || SendAll(socket, body.data(), body.size());
}

bool SendFile(int socket, const std::string& path, bool head_only) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    return SendResponse(socket, "404 Not Found", "text/plain", "not found\n",
                        head_only);
  }
  const std::streamoff size = stream.tellg();
  if (size < 0) {
    return SendResponse(socket, "500 Internal Server Error", "text/plain",
                        "read error\n", head_only);
  }
  stream.seekg(0);
  std::ostringstream header;
  header << "HTTP/1.1 200 OK\r\n"
         << "Content-Type: " << MimeType(path) << "\r\n"
         << "Content-Length: " << size << "\r\n"
         << "Cache-Control: no-cache\r\n"
         << "Access-Control-Allow-Origin: *\r\n"
         << "Connection: close\r\n\r\n";
  if (!SendAll(socket, header.str().data(), header.str().size()) || head_only) {
    return true;
  }
  std::vector<char> buffer(64 * 1024);
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = stream.gcount();
    if (count <= 0) {
      break;
    }
    if (!SendAll(socket, buffer.data(), static_cast<std::size_t>(count))) {
      return false;
    }
  }
  return true;
}

}

class LocalWebServer::Impl {
 public:
  explicit Impl(LocalWebServerConfig server_config)
      : config(std::move(server_config)) {
    if (config.port <= 0 || config.port > 65535) {
      throw std::invalid_argument("invalid HTTP server port");
    }
    if (config.web_root.empty()) {
      throw std::invalid_argument("web root must not be empty");
    }
  }

  ~Impl() { Stop(); }

  void Start() {
    if (running) {
      return;
    }
    const std::string index = config.web_root + "/index.html";
    if (!std::filesystem::exists(index)) {
      throw std::runtime_error("Lichtblick web assets not found: " + index);
    }
    listen_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_socket < 0) {
      throw std::runtime_error("failed to create HTTP socket");
    }
    const int enabled = 1;
    setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &enabled,
               sizeof(enabled));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(config.port));
    inet_pton(AF_INET, config.host.c_str(), &address.sin_addr);
    if (bind(listen_socket, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) != 0 ||
        listen(listen_socket, 16) != 0) {
      close(listen_socket);
      listen_socket = -1;
      throw std::runtime_error("failed to bind HTTP port " +
                               std::to_string(config.port));
    }
    running = true;
    thread = std::thread([this]() { Loop(); });
  }

  void Stop() {
    if (!running) {
      return;
    }
    running = false;
    if (thread.joinable()) {
      thread.join();
    }
    if (listen_socket >= 0) {
      close(listen_socket);
      listen_socket = -1;
    }
  }

  [[nodiscard]] std::string Url() const {
    return "http://" + config.host + ":" + std::to_string(config.port);
  }

  [[nodiscard]] std::string BrowserUrl() const {
    const std::string layout = Url() + "/rgbd-layout.json";
    return Url() +
           "/?ds=foxglove-websocket&ds.url=" + UrlEncode(config.websocket_url) +
           "&layoutUrl=" + UrlEncode(layout);
  }

  [[nodiscard]] int Port() const noexcept { return config.port; }

 private:
  void Loop() {
    while (running) {
      pollfd descriptor{listen_socket, POLLIN, 0};
      const int ready = poll(&descriptor, 1, 200);
      if (ready <= 0) {
        continue;
      }
      const int client = accept(listen_socket, nullptr, nullptr);
      if (client < 0) {
        continue;
      }
      timeval timeout{};
      timeout.tv_sec = 2;
      setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      HandleClient(client);
      close(client);
    }
  }

  void HandleClient(int client) {
    std::string request;
    char buffer[1024];
    while (request.find("\r\n\r\n") == std::string::npos &&
           request.size() < 8192) {
      const ssize_t count = recv(client, buffer, sizeof(buffer), 0);
      if (count <= 0) {
        return;
      }
      request.append(buffer, static_cast<std::size_t>(count));
    }
    const std::size_t line_end = request.find("\r\n");
    if (line_end == std::string::npos) {
      return;
    }
    const std::string line = request.substr(0, line_end);
    const std::size_t first_space = line.find(' ');
    const std::size_t second_space = first_space == std::string::npos
                                         ? std::string::npos
                                         : line.find(' ', first_space + 1);
    if (first_space == std::string::npos || second_space == std::string::npos) {
      return;
    }
    const std::string method = line.substr(0, first_space);
    if (method != "GET" && method != "HEAD") {
      SendResponse(client, "405 Method Not Allowed", "text/plain",
                   "method not allowed\n", false);
      return;
    }
    const bool head_only = method == "HEAD";
    std::string path =
        line.substr(first_space + 1, second_space - first_space - 1);
    const std::size_t query = path.find('?');
    if (query != std::string::npos) {
      path.erase(query);
    }
    path = UrlDecode(path);
    if (path.find("..") != std::string::npos) {
      SendResponse(client, "403 Forbidden", "text/plain", "forbidden\n",
                   head_only);
      return;
    }
    if (path.empty() || path == "/") {
      path = "/index.html";
    }
    if (path == "/rgbd-layout.json") {
      SendResponse(client, "200 OK", "application/json", kLayoutJson,
                   head_only);
      return;
    }
    if (path.front() != '/') {
      path.insert(path.begin(), '/');
    }
    SendFile(client, config.web_root + path, head_only);
  }

  LocalWebServerConfig config;
  int listen_socket = -1;
  std::atomic<bool> running{false};
  std::thread thread;
};

LocalWebServer::LocalWebServer(LocalWebServerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

LocalWebServer::~LocalWebServer() { impl_->Stop(); }

void LocalWebServer::Start() { impl_->Start(); }

void LocalWebServer::Stop() { impl_->Stop(); }

std::string LocalWebServer::Url() const { return impl_->Url(); }

std::string LocalWebServer::BrowserUrl() const { return impl_->BrowserUrl(); }

int LocalWebServer::Port() const noexcept { return impl_->Port(); }

}
