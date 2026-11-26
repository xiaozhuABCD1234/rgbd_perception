#include "rgbd/recognition/yolo26_recognizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rgbd {

namespace {

struct RawDetection {
  float x1 = 0.0F;
  float y1 = 0.0F;
  float x2 = 0.0F;
  float y2 = 0.0F;
  float score = 0.0F;
  int class_id = 0;
  std::vector<float> mask_coefficients;
};

struct Letterbox {
  Tensor tensor;
  float scale = 1.0F;
  float pad_x = 0.0F;
  float pad_y = 0.0F;
};

float Clamp(float value, float low, float high) {
  return std::max(low, std::min(value, high));
}

float IoU(const RawDetection& first, const RawDetection& second) {
  const float left = std::max(first.x1, second.x1);
  const float top = std::max(first.y1, second.y1);
  const float right = std::min(first.x2, second.x2);
  const float bottom = std::min(first.y2, second.y2);
  const float intersection =
      std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
  const float first_area =
      std::max(0.0F, first.x2 - first.x1) * std::max(0.0F, first.y2 - first.y1);
  const float second_area = std::max(0.0F, second.x2 - second.x1) *
                            std::max(0.0F, second.y2 - second.y1);
  const float union_area = first_area + second_area - intersection;
  return union_area <= 0.0F ? 0.0F : intersection / union_area;
}

std::vector<RawDetection> Nms(std::vector<RawDetection> detections,
                              float threshold) {
  std::sort(detections.begin(), detections.end(),
            [](const RawDetection& first, const RawDetection& second) {
              return first.score > second.score;
            });
  std::vector<RawDetection> kept;
  for (const RawDetection& candidate : detections) {
    bool suppressed = false;
    for (const RawDetection& selected : kept) {
      if (candidate.class_id == selected.class_id &&
          IoU(candidate, selected) > threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(candidate);
    }
  }
  return kept;
}

Letterbox PrepareInput(const FrameSet& frame, int input_width,
                       int input_height) {
  const float source_width = static_cast<float>(frame.color.width);
  const float source_height = static_cast<float>(frame.color.height);
  const float scale =
      std::min(static_cast<float>(input_width) / source_width,
               static_cast<float>(input_height) / source_height);
  const int resized_width =
      std::max(1, static_cast<int>(std::round(source_width * scale)));
  const int resized_height =
      std::max(1, static_cast<int>(std::round(source_height * scale)));
  const float pad_x = (static_cast<float>(input_width) - resized_width) / 2.0F;
  const float pad_y =
      (static_cast<float>(input_height) - resized_height) / 2.0F;
  constexpr float kPadding = 114.0F / 255.0F;
  const std::size_t plane =
      static_cast<std::size_t>(input_width) * input_height;
  std::vector<float> values(plane * 3, kPadding);

  std::vector<int> source_x(resized_width);
  std::vector<int> source_x_next(resized_width);
  std::vector<float> weight_x(resized_width);
  for (int x = 0; x < resized_width; ++x) {
    const float position = Clamp((static_cast<float>(x) + 0.5F) / scale - 0.5F,
                                 0.0F, source_width - 1.0F);
    const int first = static_cast<int>(std::floor(position));
    source_x[x] = first;
    source_x_next[x] = std::min(first + 1, static_cast<int>(source_width) - 1);
    weight_x[x] = position - static_cast<float>(first);
  }

  const std::uint8_t* pixels = frame.color.pixels.data();
  const float inverse_255 = 1.0F / 255.0F;
  const int target_pad_x = static_cast<int>(std::round(pad_x));
  const int target_pad_y = static_cast<int>(std::round(pad_y));
  for (int y = 0; y < resized_height; ++y) {
    const float source_y = Clamp((static_cast<float>(y) + 0.5F) / scale - 0.5F,
                                 0.0F, source_height - 1.0F);
    const int y0 = static_cast<int>(std::floor(source_y));
    const int y1 = std::min(y0 + 1, static_cast<int>(source_height) - 1);
    const float wy = source_y - static_cast<float>(y0);
    const std::size_t row0 =
        static_cast<std::size_t>(y0) * frame.color.width * 3;
    const std::size_t row1 =
        static_cast<std::size_t>(y1) * frame.color.width * 3;
    const std::size_t target_row =
        static_cast<std::size_t>(target_pad_y + y) * input_width;
    for (int x = 0; x < resized_width; ++x) {
      const int x0 = source_x[x];
      const int x1 = source_x_next[x];
      const float wx = weight_x[x];
      const std::uint8_t* top_left = pixels + row0 + x0 * 3;
      const std::uint8_t* top_right = pixels + row0 + x1 * 3;
      const std::uint8_t* bottom_left = pixels + row1 + x0 * 3;
      const std::uint8_t* bottom_right = pixels + row1 + x1 * 3;
      const std::size_t target_offset =
          target_row + static_cast<std::size_t>(target_pad_x + x);
      for (std::size_t channel = 0; channel < 3; ++channel) {
        const float top =
            (top_left[channel] * (1.0F - wx) + top_right[channel] * wx) *
            inverse_255;
        const float bottom =
            (bottom_left[channel] * (1.0F - wx) + bottom_right[channel] * wx) *
            inverse_255;
        values[channel * plane + target_offset] =
            top * (1.0F - wy) + bottom * wy;
      }
    }
  }
  return {Tensor::Float32({1, 3, input_height, input_width}, std::move(values)),
          scale, pad_x, pad_y};
}

}  // namespace

class Yolo26Recognizer::Impl {
 public:
  Impl(std::shared_ptr<IInferenceEngine> inference_engine, Yolo26Config config)
      : engine(std::move(inference_engine)), settings(std::move(config)) {
    if (engine == nullptr) {
      throw std::invalid_argument("YOLO26 requires an inference engine");
    }
    if (settings.input_name.empty() || settings.input_width <= 0 ||
        settings.input_height <= 0) {
      throw std::invalid_argument("YOLO26 input configuration is invalid");
    }
    if (settings.confidence_threshold < 0.0F ||
        settings.confidence_threshold > 1.0F || settings.iou_threshold < 0.0F ||
        settings.iou_threshold > 1.0F || settings.mask_threshold < 0.0F ||
        settings.mask_threshold > 1.0F) {
      throw std::invalid_argument("YOLO26 thresholds must be in [0, 1]");
    }
  }

  std::vector<Detection2D> Recognize(const FrameSet& frame) {
    if (!frame.color.Valid()) {
      return {};
    }
    Letterbox input =
        PrepareInput(frame, settings.input_width, settings.input_height);
    const TensorMap outputs =
        engine->Infer({{settings.input_name, std::move(input.tensor)}});

    const Tensor* detection_tensor = nullptr;
    const Tensor* prototype_tensor = nullptr;
    for (const auto& entry : outputs) {
      const auto& shape = entry.second.Shape();
      if (shape.size() == 4) {
        prototype_tensor = &entry.second;
      } else if (shape.size() == 2 || shape.size() == 3) {
        detection_tensor = &entry.second;
      }
    }
    if (detection_tensor == nullptr) {
      throw std::runtime_error("YOLO26 output tensor not found");
    }
    std::vector<RawDetection> detections =
        Decode(*detection_tensor, prototype_tensor);
    detections = Nms(std::move(detections), settings.iou_threshold);

    std::vector<float> prototype_values;
    if (prototype_tensor != nullptr) {
      prototype_values = prototype_tensor->AsFloat32();
    }

    std::vector<Detection2D> results;
    results.reserve(detections.size());
    for (const RawDetection& detection : detections) {
      const int left = static_cast<int>(
          std::floor(Clamp((detection.x1 - input.pad_x) / input.scale, 0.0F,
                           static_cast<float>(frame.color.width - 1))));
      const int top = static_cast<int>(
          std::floor(Clamp((detection.y1 - input.pad_y) / input.scale, 0.0F,
                           static_cast<float>(frame.color.height - 1))));
      const int right = static_cast<int>(
          std::ceil(Clamp((detection.x2 - input.pad_x) / input.scale, 0.0F,
                          static_cast<float>(frame.color.width - 1))));
      const int bottom = static_cast<int>(
          std::ceil(Clamp((detection.y2 - input.pad_y) / input.scale, 0.0F,
                          static_cast<float>(frame.color.height - 1))));
      if (right <= left || bottom <= top) {
        continue;
      }
      Detection2D result;
      result.label = Label(detection.class_id);
      result.confidence = detection.score;
      result.box = {left, top, right - left, bottom - top};
      if (prototype_tensor != nullptr && !detection.mask_coefficients.empty()) {
        result.mask = DecodeMask(frame, *prototype_tensor, prototype_values,
                                 detection, input.scale, input.pad_x,
                                 input.pad_y, left, top, right, bottom);
      }
      results.push_back(std::move(result));
    }
    return results;
  }

 private:
  std::vector<RawDetection> Decode(const Tensor& tensor,
                                   const Tensor* prototype) const {
    const auto& shape = tensor.Shape();
    int anchors = 0;
    int channels = 0;
    bool channels_first = false;
    if (shape.size() == 2) {
      anchors = static_cast<int>(shape[0]);
      channels = static_cast<int>(shape[1]);
    } else if (shape.size() == 3 && shape[0] == 1) {
      if (shape[1] < shape[2]) {
        channels = static_cast<int>(shape[1]);
        anchors = static_cast<int>(shape[2]);
        channels_first = true;
      } else {
        anchors = static_cast<int>(shape[1]);
        channels = static_cast<int>(shape[2]);
      }
    } else {
      throw std::runtime_error("unsupported YOLO26 detection shape");
    }
    const std::vector<float> values = tensor.AsFloat32();
    const auto at = [&](int anchor, int channel) {
      return channels_first
                 ? values[static_cast<std::size_t>(channel) * anchors + anchor]
                 : values[static_cast<std::size_t>(anchor) * channels +
                          channel];
    };
    const int mask_channels =
        prototype == nullptr ? 0 : static_cast<int>(prototype->Shape()[1]);
    const bool end_to_end =
        (prototype == nullptr && channels == 6 && anchors <= 1000) ||
        (prototype != nullptr && channels == 6 + mask_channels &&
         anchors <= 300);
    const int class_count = !settings.labels.empty()
                                ? static_cast<int>(settings.labels.size())
                                : std::max(1, channels - 4 - mask_channels);
    if (!end_to_end && channels < 4 + class_count + mask_channels) {
      throw std::runtime_error("YOLO26 output channels do not match labels");
    }
    std::vector<RawDetection> detections;
    for (int anchor = 0; anchor < anchors; ++anchor) {
      RawDetection detection;
      if (end_to_end) {
        detection.x1 = at(anchor, 0);
        detection.y1 = at(anchor, 1);
        detection.x2 = at(anchor, 2);
        detection.y2 = at(anchor, 3);
        detection.score = at(anchor, 4);
        detection.class_id = static_cast<int>(std::round(at(anchor, 5)));
        if (prototype != nullptr) {
          detection.mask_coefficients.reserve(mask_channels);
          for (int channel = 0; channel < mask_channels; ++channel) {
            detection.mask_coefficients.push_back(at(anchor, 6 + channel));
          }
        }
      } else {
        const float center_x = at(anchor, 0);
        const float center_y = at(anchor, 1);
        const float width = at(anchor, 2);
        const float height = at(anchor, 3);
        detection.x1 = center_x - width / 2.0F;
        detection.y1 = center_y - height / 2.0F;
        detection.x2 = center_x + width / 2.0F;
        detection.y2 = center_y + height / 2.0F;
        detection.class_id = 0;
        detection.score = 0.0F;
        for (int class_id = 0; class_id < class_count; ++class_id) {
          const float score = at(anchor, 4 + class_id);
          if (score > detection.score) {
            detection.score = score;
            detection.class_id = class_id;
          }
        }
        if (prototype != nullptr) {
          detection.mask_coefficients.reserve(mask_channels);
          for (int channel = 0; channel < mask_channels; ++channel) {
            detection.mask_coefficients.push_back(
                at(anchor, 4 + class_count + channel));
          }
        }
      }
      if (detection.score >= settings.confidence_threshold) {
        detections.push_back(std::move(detection));
      }
    }
    return detections;
  }

  Mask2D DecodeMask(const FrameSet& frame, const Tensor& prototype,
                    const std::vector<float>& values,
                    const RawDetection& detection, float scale, float pad_x,
                    float pad_y, int left, int top, int right,
                    int bottom) const {
    const auto& shape = prototype.Shape();
    if (shape.size() != 4) {
      return {};
    }
    const int channels = static_cast<int>(shape[1]);
    const int proto_height = static_cast<int>(shape[2]);
    const int proto_width = static_cast<int>(shape[3]);
    if (detection.mask_coefficients.size() <
            static_cast<std::size_t>(channels) ||
        values.size() != static_cast<std::size_t>(channels) *
                             static_cast<std::size_t>(proto_height) *
                             static_cast<std::size_t>(proto_width)) {
      return {};
    }
    const float frame_scale = std::max(scale, 1e-6F);
    const float pixels_per_proto = frame_scale *
                                   static_cast<float>(settings.input_width) /
                                   static_cast<float>(proto_width);
    const std::size_t max_stride =
        std::min(frame.color.width, frame.color.height);
    std::size_t mask_stride =
        static_cast<std::size_t>(std::max(1.0F, std::ceil(pixels_per_proto)));
    mask_stride = std::max<std::size_t>(1, std::min(mask_stride, max_stride));
    Mask2D mask;
    mask.width = frame.color.width;
    mask.height = frame.color.height;
    mask.stride = mask_stride;
    mask.values.assign(mask.StrideWidth() * mask.StrideHeight(), 0);
    const std::size_t stride_width = mask.StrideWidth();
    const float width_ratio =
        static_cast<float>(proto_width) / settings.input_width;
    const float height_ratio =
        static_cast<float>(proto_height) / settings.input_height;
    const float logit_threshold =
        std::log(settings.mask_threshold /
                 std::max(1.0F - settings.mask_threshold, 1e-6F));
    const std::size_t channel_stride =
        static_cast<std::size_t>(proto_height) * proto_width;
    const float* coefficients = detection.mask_coefficients.data();
    const int stride = static_cast<int>(mask_stride);
    for (int y = top; y < bottom; y += stride) {
      const float input_y = static_cast<float>(y) * scale + pad_y;
      const int proto_y = static_cast<int>(input_y * height_ratio);
      if (proto_y < 0 || proto_y >= proto_height) {
        continue;
      }
      const float* proto_row =
          values.data() + static_cast<std::size_t>(proto_y) * proto_width;
      const std::size_t mask_row =
          static_cast<std::size_t>(y / stride) * stride_width;
      for (int x = left; x < right; x += stride) {
        const float input_x = static_cast<float>(x) * scale + pad_x;
        const int proto_x = static_cast<int>(input_x * width_ratio);
        if (proto_x < 0 || proto_x >= proto_width) {
          continue;
        }
        const float* proto_pixel = proto_row + proto_x;
        float logit = 0.0F;
        for (int channel = 0; channel < channels; ++channel) {
          logit +=
              coefficients[channel] *
              proto_pixel[static_cast<std::size_t>(channel) * channel_stride];
        }
        if (logit >= logit_threshold) {
          mask.values[mask_row + static_cast<std::size_t>(x / stride)] = 255;
        }
      }
    }
    return mask;
  }

  std::string Label(int class_id) const {
    if (class_id >= 0 && class_id < static_cast<int>(settings.labels.size())) {
      return settings.labels[class_id];
    }
    return "class_" + std::to_string(class_id);
  }

  std::shared_ptr<IInferenceEngine> engine;
  Yolo26Config settings;
};

Yolo26Recognizer::Yolo26Recognizer(std::shared_ptr<IInferenceEngine> engine,
                                   Yolo26Config config)
    : impl_(std::make_unique<Impl>(std::move(engine), std::move(config))) {}

Yolo26Recognizer::~Yolo26Recognizer() = default;

std::vector<Detection2D> Yolo26Recognizer::Recognize(const FrameSet& frame) {
  return impl_->Recognize(frame);
}

}  // namespace rgbd
