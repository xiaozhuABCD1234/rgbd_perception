#include <chrono>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rgbd/perception_pipeline.hpp"
#include "rgbd/synthetic_camera.hpp"
#include "rgbd/version.hpp"

namespace {

class DemoDetector final : public rgbd::IObjectDetector {
public:
    [[nodiscard]] std::vector<rgbd::Detection2D> detect(const rgbd::ColorImage& color) override {
        if (!color.valid()) {
            return {};
        }
        const auto width = static_cast<int>(color.width);
        const auto height = static_cast<int>(color.height);
        return {{"demo_object", 1.0F, {width / 4, height / 4, width / 2, height / 2}}};
    }
};

int parse_frame_count(int argc, char** argv) {
    if (argc == 1) {
        return 5;
    }
    if (argc != 3 || std::string(argv[1]) != "--frames") {
        throw std::invalid_argument("usage: rgbd_demo [--frames N]");
    }

    std::size_t parsed = 0;
    const std::string value(argv[2]);
    const int count = std::stoi(value, &parsed);
    if (parsed != value.size() || count <= 0) {
        throw std::invalid_argument("frame count must be a positive integer");
    }
    return count;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const int frame_count = parse_frame_count(argc, argv);
        rgbd::SyntheticCameraConfig config;
        config.frame_period = std::chrono::milliseconds(0);
        rgbd::SyntheticCamera camera(config);
        DemoDetector detector;
        rgbd::CenterPatchDepthEstimator depth_estimator;
        rgbd::PerceptionPipeline pipeline(detector, depth_estimator);

        std::cout << "rgbd_perception " << rgbd::kVersion << " (synthetic data, no real inference)\n";
        camera.open();
        camera.start();

        for (int index = 0; index < frame_count; ++index) {
            rgbd::FrameSet frame;
            if (!camera.read(frame, std::chrono::milliseconds(100))) {
                throw std::runtime_error("failed to read synthetic frame");
            }
            const auto observations = pipeline.process(frame);
            for (const auto& observation : observations) {
                std::cout << "frame=" << observation.sequence << " label=" << observation.detection.label
                          << " depth_quality=" << rgbd::to_string(observation.depth.quality);
                if (observation.depth.distance_m) {
                    std::cout << std::fixed << std::setprecision(3)
                              << " distance_m=" << *observation.depth.distance_m;
                }
                std::cout << '\n';
            }
        }

        camera.stop();
        camera.close();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
