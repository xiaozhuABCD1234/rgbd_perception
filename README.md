# rgbd_perception

面向 Intel RealSense D435i、Orbbec Gemini 305、合成数据与录制数据的 ROS-free C++17 RGB-D 感知框架。

项目提供统一相机接口，将厂商 SDK 隔离在可选后端库中，并输出目标在相机坐标系下的三维位置。

## 分层结构

```text
apps/                 命令行入口
include/rgbd/core/    帧类型、接口、鲁棒深度融合、感知管线
src/core/             核心实现
include/rgbd/inference/
src/inference/        推理运行时抽象与 Tensor 容器
include/rgbd/recognition/
src/recognition/      与模型相关的解码与识别逻辑
include/rgbd/io/
src/io/               录制与回放适配器
include/rgbd/cameras/
src/cameras/          合成相机、Gemini 305、D435i 与相机工厂
```

依赖方向：

```text
apps -> cameras -> io -> core
apps -> recognition -> inference -> core
cameras/orbbec -> core
cameras/realsense -> core
```

公共头文件不暴露任何厂商 SDK 类型。ONNX Runtime、TensorRT、OpenVINO 适配器放在 `src/inference/`，实现 `rgbd::IInferenceEngine`。YOLO、DETR、分割模型解码器放在 `src/recognition/`，实现 `rgbd::IVisionRecognizer`。

## 环境要求

- CMake 3.20 或更高版本
- Ninja
- 支持 C++17 的 GCC 或 Clang
- cpplint
- clang-format
- 可选：Orbbec SDK v2
- 可选：librealsense2

## 构建

```bash
just hooks
just build
just test
just check
```

等价的 CMake 命令：

```bash
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev
```

其他预设：

```bash
just release
just sanitizer
just gemini305
just d435i
```

## 相机 URI

```text
synthetic://?width=640&height=480&depth_m=2.0&fps=30
replay:///path/to/session.rgbd?loop=true
gemini305://?serial=...&width=640&height=480&fps=30
d435i://?serial=...&width=640&height=480&fps=30
```

`gemini305` 需要 `RGBD_WITH_ORBBEC=ON`。`d435i` 需要 `RGBD_WITH_REALSENSE=ON`。

## 应用

```bash
rgbd_demo --frames 10
rgbd_demo --camera "gemini305://?width=640&height=480&fps=30" --frames 100
rgbd_viewer "d435i://?width=640&height=480&fps=30" --frames 100
rgbd_replay build/demo.rgbd
```

`rgbd_demo` 和 `rgbd_viewer` 支持 `--record PATH`。当前录制使用测试与回放共用的内部 `.rgbd` 格式。IO 层已隔离，后续可以用 MCAP 读写器替换，不影响相机、推理和识别代码。Lichtblick 可视化计划通过 MCAP 适配器接入。

## 识别与三维定位

`IVisionRecognizer` 接收完整的 `FrameSet`，因此纯彩色检测器、分割模型以及未来的 RGB-D 端到端模型可以共用同一接口。

每个检测结果可以包含二维框和可选掩码。`RobustDepthFusionEstimator` 对检测区域采样深度，过滤无效值，使用中值绝对偏差剔除离群点，融合有效深度，再通过 `CameraIntrinsics` 反投影为相机坐标系下的 `(x, y, z)`。

## 仓库与提交规范

提交信息遵循 Conventional Commits，由 `.githooks/commit-msg` 和 `.githooks/check-commit-range` 校验。

```text
feat(camera): add D435i backend
fix(depth): reject invalid ROI
test(replay): cover loop playback
build: enable cpplint
```

## 依赖

`third_party/OrbbecSDK` 已初始化为 Git 子模块。`third_party/librealsense` 和 `third_party/mcap` 已在 `.gitmodules` 中声明为可选子模块。网络可用时添加或更新：

```bash
git submodule add -f https://github.com/IntelRealSense/librealsense.git third_party/librealsense
git submodule add -f https://github.com/foxglove/mcap.git third_party/mcap
git submodule update --init --recursive
```
