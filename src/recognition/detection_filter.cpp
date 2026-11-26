#include "rgbd/recognition/detection_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

struct Track {
  std::string label;
  Rect box;
  float confidence = 0.0F;
  std::size_t hits = 0;
  std::size_t missed = 0;
};

float IntersectionOverUnion(const Rect& first, const Rect& second) {
  const int left = std::max(first.x, second.x);
  const int top = std::max(first.y, second.y);
  const int right = std::min(first.x + first.width, second.x + second.width);
  const int bottom = std::min(first.y + first.height, second.y + second.height);
  const int intersection_width = std::max(0, right - left);
  const int intersection_height = std::max(0, bottom - top);
  const float intersection = static_cast<float>(intersection_width) *
                             static_cast<float>(intersection_height);
  const float first_area =
      static_cast<float>(first.width) * static_cast<float>(first.height);
  const float second_area =
      static_cast<float>(second.width) * static_cast<float>(second.height);
  const float union_area = first_area + second_area - intersection;
  return union_area <= 0.0F ? 0.0F : intersection / union_area;
}

Rect Blend(const Rect& previous, const Rect& current, float alpha) {
  const auto blend = [alpha](int old_value, int new_value) {
    return static_cast<int>(
        std::lround(static_cast<float>(old_value) * (1.0F - alpha) +
                    static_cast<float>(new_value) * alpha));
  };
  Rect result;
  result.x = blend(previous.x, current.x);
  result.y = blend(previous.y, current.y);
  result.width = std::max(1, blend(previous.width, current.width));
  result.height = std::max(1, blend(previous.height, current.height));
  return result;
}

}

class FilteringRecognizer::Impl {
 public:
  Impl(std::unique_ptr<IVisionRecognizer> source,
       DetectionFilterConfig filter_config) {
    recognizer = std::move(source);
    settings = filter_config;
    if (recognizer == nullptr) {
      throw std::invalid_argument("detection filter requires a recognizer");
    }
    if (settings.min_hits == 0) {
      throw std::invalid_argument("min_hits must be greater than zero");
    }
    if (!std::isfinite(settings.smoothing) || settings.smoothing < 0.0F ||
        settings.smoothing > 1.0F) {
      throw std::invalid_argument("smoothing must be in [0, 1]");
    }
    if (!std::isfinite(settings.min_iou) || settings.min_iou < 0.0F ||
        settings.min_iou > 1.0F) {
      throw std::invalid_argument("min_iou must be in [0, 1]");
    }
    if (settings.max_tracks == 0) {
      throw std::invalid_argument("max_tracks must be greater than zero");
    }
  }

  std::vector<Detection2D> Recognize(const FrameSet& frame) {
    std::vector<Detection2D> detections = recognizer->Recognize(frame);
    std::vector<bool> matched(tracks.size(), false);
    std::vector<Detection2D> output;
    output.reserve(detections.size());

    for (Detection2D& detection : detections) {
      if (!detection.box.Valid()) {
        continue;
      }
      std::size_t best = tracks.size();
      float best_iou = settings.min_iou;
      for (std::size_t index = 0; index < tracks.size(); ++index) {
        if (matched[index] || tracks[index].label != detection.label) {
          continue;
        }
        const float iou =
            IntersectionOverUnion(tracks[index].box, detection.box);
        if (iou > best_iou) {
          best_iou = iou;
          best = index;
        }
      }

      if (best == tracks.size()) {
        if (tracks.size() >= settings.max_tracks) {
          continue;
        }
        Track track;
        track.label = detection.label;
        track.box = detection.box;
        track.confidence = detection.confidence;
        track.hits = 1;
        tracks.push_back(std::move(track));
        best = tracks.size() - 1;
        matched.push_back(true);
      } else {
        Track& track = tracks[best];
        track.box = settings.smoothing > 0.0F
                        ? Blend(track.box, detection.box, settings.smoothing)
                        : detection.box;
        track.confidence =
            settings.smoothing > 0.0F
                ? track.confidence * (1.0F - settings.smoothing) +
                      detection.confidence * settings.smoothing
                : detection.confidence;
        ++track.hits;
        track.missed = 0;
      }
      matched[best] = true;

      if (tracks[best].hits >= settings.min_hits) {
        Detection2D filtered = std::move(detection);
        filtered.box = tracks[best].box;
        filtered.confidence = tracks[best].confidence;
        filtered.label = tracks[best].label;
        output.push_back(std::move(filtered));
      }
    }

    for (std::size_t index = 0; index < tracks.size(); ++index) {
      if (!matched[index]) {
        ++tracks[index].missed;
      }
    }
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                                [this](const Track& track) {
                                  return track.missed > settings.max_missed;
                                }),
                 tracks.end());
    return output;
  }

 private:
  std::unique_ptr<IVisionRecognizer> recognizer;
  DetectionFilterConfig settings;
  std::vector<Track> tracks;
};

FilteringRecognizer::FilteringRecognizer(
    std::unique_ptr<IVisionRecognizer> recognizer, DetectionFilterConfig config)
    : impl_(std::make_unique<Impl>(std::move(recognizer), config)) {}

FilteringRecognizer::~FilteringRecognizer() = default;

std::vector<Detection2D> FilteringRecognizer::Recognize(const FrameSet& frame) {
  return impl_->Recognize(frame);
}

}
