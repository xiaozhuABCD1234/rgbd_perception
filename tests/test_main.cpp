#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rgbd/perception_pipeline.hpp"
#include "rgbd/synthetic_camera.hpp"

namespace {

void check(bool condition, const char* expression, const char* file, int line) {
    if (!condition) {
        throw std::runtime_error(std::string(file) + ":" + std::to_string(line) +
                                 ": check failed: " + expression);
    }
}

#define RGBD_CHECK(expression) check((expression), #expression, __FILE__, __LINE__)

rgbd::FrameSet make_frame() {
    constexpr std::size_t width = 64;
    constexpr std::size_t height = 48;
    rgbd::FrameSet frame;
    frame.camera_id = "test_camera";
    frame.sequence = 7;
    frame.device_timestamp = std::chrono::nanoseconds(123456);
    frame.color.width = width;
    frame.color.height = height;
    frame.color.pixels.assign(width * height * rgbd::ColorImage::channels, 0);
    frame.depth.width = width;
    frame.depth.height = height;
    frame.depth.values.assign(width * height, 2.0F);
    frame.color_intrinsics = {width, height, 50.0F, 50.0F, 31.5F, 23.5F};
    return frame;
}

void test_median_depth_and_position() {
    auto frame = make_frame();
    frame.depth.values[0] = std::numeric_limits<float>::quiet_NaN();
    frame.depth.values[1] = 100.0F;
    const rgbd::CenterPatchDepthEstimator estimator;
    const rgbd::Detection2D detection{"object", 0.9F, {16, 12, 32, 24}};

    const auto estimate = estimator.estimate(frame, detection);
    RGBD_CHECK(estimate.quality == rgbd::DepthQuality::valid);
    RGBD_CHECK(estimate.distance_m.has_value());
    RGBD_CHECK(std::fabs(*estimate.distance_m - 2.0F) < 1e-6F);
    RGBD_CHECK(estimate.position_camera_m.has_value());
    RGBD_CHECK(std::fabs(estimate.position_camera_m->x - 0.02F) < 1e-6F);
    RGBD_CHECK(std::fabs(estimate.position_camera_m->y - 0.02F) < 1e-6F);
    RGBD_CHECK(std::fabs(estimate.position_camera_m->z - 2.0F) < 1e-6F);
}

void test_invalid_depth_is_explicit() {
    auto frame = make_frame();
    frame.depth.values.assign(frame.depth.values.size(), std::numeric_limits<float>::quiet_NaN());
    const rgbd::CenterPatchDepthEstimator estimator;
    const auto estimate = estimator.estimate(frame, {"object", 0.9F, {16, 12, 32, 24}});

    RGBD_CHECK(estimate.quality == rgbd::DepthQuality::too_few_valid_pixels);
    RGBD_CHECK(!estimate.distance_m.has_value());
    RGBD_CHECK(!estimate.position_camera_m.has_value());
}

void test_roi_is_clipped_to_image() {
    const auto frame = make_frame();
    const rgbd::CenterPatchDepthEstimator estimator;
    const auto estimate = estimator.estimate(frame, {"edge", 0.8F, {-8, -6, 16, 12}});

    RGBD_CHECK(estimate.quality == rgbd::DepthQuality::valid);
    RGBD_CHECK(estimate.distance_m.has_value());
    RGBD_CHECK(std::fabs(*estimate.distance_m - 2.0F) < 1e-6F);
}

class StaticDetector final : public rgbd::IObjectDetector {
public:
    [[nodiscard]] std::vector<rgbd::Detection2D> detect(const rgbd::ColorImage&) override {
        return {{"box", 0.75F, {16, 12, 32, 24}}};
    }
};

void test_pipeline_preserves_frame_identity() {
    const auto frame = make_frame();
    StaticDetector detector;
    const rgbd::CenterPatchDepthEstimator estimator;
    const rgbd::PerceptionPipeline pipeline(detector, estimator);

    const auto observations = pipeline.process(frame);
    RGBD_CHECK(observations.size() == 1);
    RGBD_CHECK(observations.front().camera_id == frame.camera_id);
    RGBD_CHECK(observations.front().sequence == frame.sequence);
    RGBD_CHECK(observations.front().timestamp == frame.device_timestamp);
    RGBD_CHECK(observations.front().depth.quality == rgbd::DepthQuality::valid);
}

void test_camera_lifecycle() {
    rgbd::SyntheticCameraConfig config;
    config.width = 16;
    config.height = 12;
    config.frame_period = std::chrono::milliseconds(0);
    rgbd::SyntheticCamera camera(config);
    rgbd::FrameSet frame;

    RGBD_CHECK(!camera.read(frame, std::chrono::milliseconds(0)));
    camera.open();
    camera.start();
    RGBD_CHECK(camera.read(frame, std::chrono::milliseconds(1)));
    RGBD_CHECK(frame.valid());
    RGBD_CHECK(frame.sequence == 0);
    RGBD_CHECK(camera.read(frame, std::chrono::milliseconds(1)));
    RGBD_CHECK(frame.sequence == 1);
    camera.stop();
    RGBD_CHECK(!camera.read(frame, std::chrono::milliseconds(0)));
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"median depth and position", test_median_depth_and_position},
        {"invalid depth is explicit", test_invalid_depth_is_explicit},
        {"ROI clipping", test_roi_is_clipped_to_image},
        {"pipeline frame identity", test_pipeline_preserves_frame_identity},
        {"camera lifecycle", test_camera_lifecycle},
    };

    int failures = 0;
    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "[PASS] " << test.first << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[FAIL] " << test.first << ": " << error.what() << '\n';
        }
    }

    std::cout << tests.size() - static_cast<std::size_t>(failures) << "/" << tests.size()
              << " tests passed\n";
    return failures == 0 ? 0 : 1;
}
