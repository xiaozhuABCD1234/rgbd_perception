#include "rgbd/perception_pipeline.hpp"

#include <utility>
#include <vector>

namespace rgbd {

PerceptionPipeline::PerceptionPipeline(
    IObjectDetector& detector, const IDepthEstimator& depth_estimator) noexcept
    : detector_(detector), depth_estimator_(depth_estimator) {}

std::vector<ObjectObservation> PerceptionPipeline::process(
    const FrameSet& frame) const {
  auto detections = detector_.detect(frame.color);
  std::vector<ObjectObservation> observations;
  observations.reserve(detections.size());

  for (auto& detection : detections) {
    ObjectObservation observation;
    observation.camera_id = frame.camera_id;
    observation.sequence = frame.sequence;
    observation.timestamp = frame.device_timestamp;
    observation.depth = depth_estimator_.estimate(frame, detection);
    observation.detection = std::move(detection);
    observations.push_back(std::move(observation));
  }
  return observations;
}

}  // namespace rgbd
