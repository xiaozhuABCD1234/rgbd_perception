#include "rgbd/core/perception_pipeline.hpp"

#include <utility>
#include <vector>

namespace rgbd {

PerceptionPipeline::PerceptionPipeline(
    IVisionRecognizer& recognizer, const IDepthEstimator& depth_estimator,
    const IObjectGeometryEstimator* geometry_estimator) noexcept {
  recognizer_ = &recognizer;
  depth_estimator_ = &depth_estimator;
  geometry_estimator_ = geometry_estimator;
}

std::vector<ObjectObservation> PerceptionPipeline::Process(
    const FrameSet& frame) const {
  auto detections = recognizer_->Recognize(frame);
  std::vector<ObjectObservation> observations;
  observations.reserve(detections.size());

  for (auto& detection : detections) {
    ObjectObservation observation;
    observation.camera_id = frame.camera_id;
    observation.sequence = frame.sequence;
    observation.timestamp = frame.device_timestamp;
    observation.depth = depth_estimator_->Estimate(frame, detection);
    if (geometry_estimator_ != nullptr) {
      observation.geometry = geometry_estimator_->Estimate(frame, detection);
    }
    observation.detection = std::move(detection);
    observations.push_back(std::move(observation));
  }
  return observations;
}

}
