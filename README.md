# rgbd_perception

ROS-free C++17 RGB-D perception framework for Intel RealSense D435i, Orbbec Gemini 305, synthetic data and recorded data.

The project exposes a single camera interface, keeps vendor SDKs behind optional backend libraries, and returns object positions in the camera coordinate frame.

## Layers

```text
apps/                 command line entry points
include/rgbd/core/    frame types, interfaces, robust depth fusion, pipeline
src/core/             core implementation
include/rgbd/inference/
src/inference/        model-runtime abstraction and tensor container
include/rgbd/recognition/
src/recognition/      model-specific decoding and recognition logic
include/rgbd/io/
src/io/               recording and replay adapters
include/rgbd/cameras/
src/cameras/          synthetic, Gemini 305, D435i and camera factory
```

Dependency direction:

```text
apps -> cameras -> io -> core
apps -> recognition -> inference -> core
cameras/orbbec -> core
cameras/realsense -> core
```

No vendor SDK type appears in a public header. ONNX Runtime, TensorRT and OpenVINO adapters belong under `src/inference/` and are expected to implement `rgbd::IInferenceEngine`. YOLO, DETR and segmentation decoders belong under `src/recognition/` and implement `rgbd::IVisionRecognizer`.

## Requirements

- CMake 3.20 or newer
- Ninja
- GCC or Clang with C++17
- cpplint
- clang-format
- Optional: Orbbec SDK v2
- Optional: librealsense2

## Build

```bash
just hooks
just build
just test
just check
```

Equivalent manual commands:

```bash
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev
```

Other presets:

```bash
just release
just sanitizer
just gemini305
just d435i
```

## Camera URIs

```text
synthetic://?width=640&height=480&depth_m=2.0&fps=30
replay:///path/to/session.rgbd?loop=true
gemini305://?serial=...&width=640&height=480&fps=30
d435i://?serial=...&width=640&height=480&fps=30
```

`gemini305` requires `RGBD_WITH_ORBBEC=ON`. `d435i` requires `RGBD_WITH_REALSENSE=ON`.

## Applications

```bash
rgbd_demo --frames 10
rgbd_demo --camera "gemini305://?width=640&height=480&fps=30" --frames 100
rgbd_viewer "d435i://?width=640&height=480&fps=30" --frames 100
rgbd_replay build/demo.rgbd
```

`rgbd_demo` and `rgbd_viewer` support `--record PATH`. Recordings currently use the internal `.rgbd` transport used by tests and replay. The IO layer is isolated so an MCAP writer and reader can replace it without touching camera, inference or recognition code. Lichtblick visualization is intended to consume the MCAP adapter.

## Recognition and localization

`IVisionRecognizer` receives the complete `FrameSet`, so color-only detectors, segmentation models and future RGB-D end-to-end models use the same interface.

Each detection may contain a box and an optional mask. `RobustDepthFusionEstimator` samples the detection region, rejects invalid depth, applies median absolute deviation filtering, fuses valid samples and unprojects the result through `CameraIntrinsics` into camera-frame `(x, y, z)`.

## Repository and commits

Commit messages follow Conventional Commits and are enforced by `.githooks/commit-msg` and `.githooks/check-commit-range`.

```text
feat(camera): add D435i backend
fix(depth): reject invalid ROI
test(replay): cover loop playback
build: enable cpplint
```

## Dependencies

`third_party/OrbbecSDK` is an initialized Git submodule. `third_party/librealsense` and `third_party/mcap` are declared in `.gitmodules` as optional submodules. Add or refresh them when the remotes are available:

```bash
git submodule add -f https://github.com/IntelRealSense/librealsense.git third_party/librealsense
git submodule add -f https://github.com/foxglove/mcap.git third_party/mcap
git submodule update --init --recursive
```
