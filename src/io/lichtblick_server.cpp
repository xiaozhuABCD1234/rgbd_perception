#include "rgbd/io/lichtblick_server.hpp"

#include <libwebsockets.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "foxglove_messages.hpp"

namespace rgbd {

namespace {

constexpr std::uint64_t kColorChannel = 1;
constexpr std::uint64_t kDepthChannel = 2;
constexpr std::uint64_t kCalibrationChannel = 3;

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string Base64(const std::string& input) {
  static constexpr char kTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve((input.size() + 2) / 3 * 4);
  std::size_t index = 0;
  while (index + 2 < input.size()) {
    const auto first = static_cast<unsigned char>(input[index]);
    const auto second = static_cast<unsigned char>(input[index + 1]);
    const auto third = static_cast<unsigned char>(input[index + 2]);
    output.push_back(kTable[first >> 2]);
    output.push_back(kTable[((first & 0x03U) << 4U) | (second >> 4U)]);
    output.push_back(kTable[((second & 0x0FU) << 2U) | (third >> 6U)]);
    output.push_back(kTable[third & 0x3FU]);
    index += 3;
  }
  if (index < input.size()) {
    const auto first = static_cast<unsigned char>(input[index]);
    output.push_back(kTable[first >> 2]);
    if (index + 1 < input.size()) {
      const auto second = static_cast<unsigned char>(input[index + 1]);
      output.push_back(kTable[((first & 0x03U) << 4U) | (second >> 4U)]);
      output.push_back(kTable[(second & 0x0FU) << 2U]);
      output.push_back('=');
    } else {
      output.push_back(kTable[(first & 0x03U) << 4U]);
      output.push_back('=');
      output.push_back('=');
    }
  }
  return output;
}

std::string ChannelJson(std::uint64_t id, const std::string& topic,
                        const std::string& schema_name,
                        const std::string& schema_data) {
  nlohmann::json channel;
  channel["id"] = id;
  channel["topic"] = topic;
  channel["encoding"] = "protobuf";
  channel["schemaName"] = schema_name;
  channel["schemaEncoding"] = "protobuf";
  channel["schema"] = Base64(schema_data);
  return channel.dump();
}

struct Outgoing {
  bool binary = false;
  std::vector<unsigned char> data;
};

struct SessionState {
  lws* wsi = nullptr;
  std::map<std::uint64_t, std::set<std::uint32_t>> subscriptions;
  std::deque<Outgoing> queue;
};

struct PerSession {
  SessionState* state = nullptr;
};

std::vector<unsigned char> BinaryMessage(std::uint32_t subscription_id,
                                         std::int64_t timestamp,
                                         const std::string& payload) {
  std::vector<unsigned char> message;
  message.reserve(13 + payload.size());
  message.push_back(1);
  for (std::size_t index = 0; index < 4; ++index) {
    message.push_back(
        static_cast<unsigned char>((subscription_id >> (index * 8U)) & 0xFFU));
  }
  const auto time = static_cast<std::uint64_t>(timestamp);
  for (std::size_t index = 0; index < 8; ++index) {
    message.push_back(
        static_cast<unsigned char>((time >> (index * 8U)) & 0xFFU));
  }
  message.insert(message.end(), payload.begin(), payload.end());
  return message;
}

}

class LichtblickServer::Impl {
 public:
  explicit Impl(LichtblickServerConfig server_config)
      : config(std::move(server_config)) {
    if (config.port <= 0 || config.port > 65535) {
      throw std::invalid_argument("invalid Lichtblick server port");
    }
    const std::string raw_schema = internal::FoxgloveRawImageDescriptor();
    const std::string calibration_schema =
        internal::FoxgloveCameraCalibrationDescriptor();
    nlohmann::json server_info;
    server_info["op"] = "serverInfo";
    server_info["name"] = config.name;
    server_info["capabilities"] = nlohmann::json::array();
    server_info_json = server_info.dump();
    nlohmann::json advertise;
    advertise["op"] = "advertise";
    advertise["channels"] = nlohmann::json::array(
        {nlohmann::json::parse(ChannelJson(kColorChannel,
                                           "/camera/color/image_raw",
                                           "foxglove.RawImage", raw_schema)),
         nlohmann::json::parse(ChannelJson(kDepthChannel,
                                           "/camera/depth/image_raw",
                                           "foxglove.RawImage", raw_schema)),
         nlohmann::json::parse(
             ChannelJson(kCalibrationChannel, "/camera/color/camera_info",
                         "foxglove.CameraCalibration", calibration_schema))});
    advertise_json = advertise.dump();
  }

  ~Impl() { Stop(); }

  void Start() {
    if (context != nullptr) {
      return;
    }
    lws_set_log_level(LLL_ERR, nullptr);
    lws_context_creation_info info{};
    info.port = config.port;
    info.protocols = ProtocolTable();
    info.gid = -1;
    info.uid = -1;
    info.user = this;
    context = lws_create_context(&info);
    if (context == nullptr) {
      throw std::runtime_error("failed to start Lichtblick WebSocket server");
    }
    running = true;
    service_thread = std::thread([this]() {
      while (running) {
        lws_service(context, 50);
      }
    });
  }

  void Stop() {
    if (context == nullptr) {
      return;
    }
    running = false;
    lws_cancel_service(context);
    if (service_thread.joinable()) {
      service_thread.join();
    }
    lws_context_destroy(context);
    context = nullptr;
    std::lock_guard<std::mutex> lock(mutex);
    for (SessionState* state : sessions) {
      delete state;
    }
    sessions.clear();
  }

  void Publish(const FrameSet& frame) {
    if (!frame.Valid()) {
      throw std::invalid_argument("cannot publish an invalid frame");
    }
    const std::int64_t timestamp = NowNs();
    const std::string color_payload = internal::SerializeFoxgloveRawImage(
        frame, timestamp, config.frame_id, "rgb8",
        static_cast<std::uint32_t>(frame.color.width * ColorImage::channels),
        frame.color.pixels.data(), frame.color.pixels.size());
    std::vector<std::uint16_t> depth_mm(frame.depth.values.size());
    for (std::size_t index = 0; index < frame.depth.values.size(); ++index) {
      const float meters = frame.depth.values[index];
      std::uint16_t value = 0;
      if (meters >= config.min_depth_m && meters <= config.max_depth_m) {
        const float scaled = meters * 1000.0F;
        value = scaled >= 65535.0F ? 65535U
                                   : static_cast<std::uint16_t>(scaled + 0.5F);
      }
      depth_mm[index] = value;
    }
    const std::string depth_payload = internal::SerializeFoxgloveRawImage(
        frame, timestamp, config.frame_id, "mono16",
        static_cast<std::uint32_t>(frame.depth.width * sizeof(std::uint16_t)),
        depth_mm.data(), depth_mm.size() * sizeof(std::uint16_t));
    const std::string calibration_payload =
        internal::SerializeFoxgloveCameraCalibration(frame, timestamp,
                                                     config.frame_id);

    bool queued = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (sessions.empty()) {
        return;
      }
      for (SessionState* state : sessions) {
        QueueSubscriptions(state, kColorChannel, timestamp, color_payload);
        QueueSubscriptions(state, kDepthChannel, timestamp, depth_payload);
        QueueSubscriptions(state, kCalibrationChannel, timestamp,
                           calibration_payload);
        queued = queued || !state->queue.empty();
      }
    }
    if (queued) {
      lws_cancel_service(context);
    }
  }

  [[nodiscard]] std::string Url() const {
    return "ws://localhost:" + std::to_string(config.port);
  }

  [[nodiscard]] int Port() const noexcept { return config.port; }

  int Callback(lws* wsi, lws_callback_reasons reason, void* user, void* in,
               std::size_t len) {
    auto* session = static_cast<PerSession*>(user);
    switch (reason) {
      case LWS_CALLBACK_ESTABLISHED:
        return OnEstablished(wsi, session);
      case LWS_CALLBACK_CLOSED:
        return OnClosed(session);
      case LWS_CALLBACK_RECEIVE:
        return OnReceive(wsi, session, in, len);
      case LWS_CALLBACK_SERVER_WRITEABLE:
        return OnWritable(session);
      case LWS_CALLBACK_EVENT_WAIT_CANCELLED:
        return OnCancelled();
      default:
        return 0;
    }
  }

 private:
  static const lws_protocols* ProtocolTable() {
    static const lws_protocols protocols[] = {
        {"foxglove.sdk.v1", &LichtblickServer::Impl::ProtocolCallback,
         sizeof(PerSession), 0, 0, nullptr, 0},
        {"foxglove.websocket.v1", &LichtblickServer::Impl::ProtocolCallback,
         sizeof(PerSession), 0, 0, nullptr, 0},
        {nullptr, nullptr, 0, 0, 0, nullptr, 0}};
    return protocols;
  }

  static int ProtocolCallback(lws* wsi, lws_callback_reasons reason, void* user,
                              void* in, std::size_t len) {
    auto* impl = static_cast<Impl*>(lws_context_user(lws_get_context(wsi)));
    return impl == nullptr ? 0 : impl->Callback(wsi, reason, user, in, len);
  }

  int OnEstablished(lws* wsi, PerSession* session) {
    auto* state = new SessionState();
    state->wsi = wsi;
    state->queue.push_back(
        {false, std::vector<unsigned char>(server_info_json.begin(),
                                           server_info_json.end())});
    state->queue.push_back(
        {false, std::vector<unsigned char>(advertise_json.begin(),
                                           advertise_json.end())});
    session->state = state;
    {
      std::lock_guard<std::mutex> lock(mutex);
      sessions.push_back(state);
    }
    lws_callback_on_writable(wsi);
    return 0;
  }

  int OnClosed(PerSession* session) {
    if (session == nullptr || session->state == nullptr) {
      return 0;
    }
    SessionState* state = session->state;
    session->state = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (auto iterator = sessions.begin(); iterator != sessions.end();
           ++iterator) {
        if (*iterator == state) {
          sessions.erase(iterator);
          break;
        }
      }
    }
    delete state;
    return 0;
  }

  int OnReceive(lws* wsi, PerSession* session, void* in, std::size_t len) {
    if (session == nullptr || session->state == nullptr || in == nullptr ||
        len == 0 || lws_frame_is_binary(wsi) != 0) {
      return 0;
    }
    try {
      const auto message =
          nlohmann::json::parse(std::string(static_cast<char*>(in), len));
      const std::string operation = message.value("op", "");
      std::lock_guard<std::mutex> lock(mutex);
      if (operation == "subscribe") {
        for (const auto& subscription : message.at("subscriptions")) {
          const auto channel_id = subscription.at("channelId").get<uint64_t>();
          const auto subscription_id = subscription.at("id").get<uint32_t>();
          session->state->subscriptions[channel_id].insert(subscription_id);
        }
      } else if (operation == "unsubscribe") {
        for (const auto& subscription_id : message.at("subscriptionIds")) {
          const auto id = subscription_id.get<uint32_t>();
          for (auto& entry : session->state->subscriptions) {
            entry.second.erase(id);
          }
        }
      }
    } catch (const std::exception&) {
      return 0;
    }
    return 0;
  }

  int OnCancelled() {
    std::lock_guard<std::mutex> lock(mutex);
    for (SessionState* state : sessions) {
      if (!state->queue.empty()) {
        lws_callback_on_writable(state->wsi);
      }
    }
    return 0;
  }

  int OnWritable(PerSession* session) {
    if (session == nullptr || session->state == nullptr) {
      return 0;
    }
    Outgoing message;
    bool has_more = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (session->state->queue.empty()) {
        return 0;
      }
      message = std::move(session->state->queue.front());
      session->state->queue.pop_front();
      has_more = !session->state->queue.empty();
    }
    const int flags = message.binary ? LWS_WRITE_BINARY : LWS_WRITE_TEXT;
    std::vector<unsigned char> buffer(LWS_PRE + message.data.size());
    std::copy(message.data.begin(), message.data.end(),
              buffer.begin() + LWS_PRE);
    if (lws_write(session->state->wsi, buffer.data() + LWS_PRE,
                  message.data.size(),
                  static_cast<lws_write_protocol>(flags)) < 0) {
      return -1;
    }
    if (has_more) {
      lws_callback_on_writable(session->state->wsi);
    }
    return 0;
  }

  void QueueSubscriptions(SessionState* state, std::uint64_t channel_id,
                          std::int64_t timestamp, const std::string& payload) {
    const auto found = state->subscriptions.find(channel_id);
    if (found == state->subscriptions.end()) {
      return;
    }
    for (const std::uint32_t subscription_id : found->second) {
      state->queue.push_back(
          {true, BinaryMessage(subscription_id, timestamp, payload)});
    }
  }

  LichtblickServerConfig config;
  lws_context* context = nullptr;
  std::thread service_thread;
  std::atomic<bool> running{false};
  std::mutex mutex;
  std::vector<SessionState*> sessions;
  std::string server_info_json;
  std::string advertise_json;
};

LichtblickServer::LichtblickServer(LichtblickServerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

LichtblickServer::~LichtblickServer() { impl_->Stop(); }

void LichtblickServer::Start() { impl_->Start(); }

void LichtblickServer::Stop() { impl_->Stop(); }

void LichtblickServer::Publish(const FrameSet& frame) { impl_->Publish(frame); }

std::string LichtblickServer::Url() const { return impl_->Url(); }

int LichtblickServer::Port() const noexcept { return impl_->Port(); }

}
