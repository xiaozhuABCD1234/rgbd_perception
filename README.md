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
- spdlog
- yaml-cpp
- 可选：Orbbec SDK v2
- 可选：librealsense2
- 可选：MCAP 子模块、Protobuf、nlohmann-json、libwebsockets，用于 Lichtblick 输出

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
gemini305://?serial=...&width=848&height=530&fps=30
d435i://?serial=...&width=640&height=480&fps=30
```

`gemini305` 需要 `RGBD_WITH_ORBBEC=ON`。`d435i` 需要 `RGBD_WITH_REALSENSE=ON`。URI 只用于命令行临时覆盖，常规参数在 YAML 中配置。

## 配置

默认读取 `config/rgbd.yaml`，也可以用 `--config PATH` 指定其他文件。命令行参数会覆盖 YAML 中的同名字段。

```yaml
camera:
  backend: gemini305
  id: gemini305
  serial: ""
  depth:
    width: 848
    height: 530
    fps: 30
    format: y16
    auto_exposure: true
    exposure_us: 15600
    gain: 16
    disparity_search_range: 128
    ae_max_exposure_us: 30158
    min_depth_m: 0.04
    max_depth_m: 10.0
  color:
    width: 848
    height: 530
    fps: 30
    format: rgb8
  align_depth_to_color: true
  timeout_ms: 1000
```

YAML 只保存深度相机参数，由 `rgbd_viewer` 和 `rgbd_lichtblick` 共用。Gemini 305 默认使用手册中的 848 x 530 @ 30fps、深度 Y16、彩色 RGB8、D2C 对齐、自动曝光、增益 16 和视差搜索范围 128；`min_depth_m` / `max_depth_m` 控制深度过滤范围，超出范围的深度会在 Lichtblick 推流和 MCAP 录制时置 0；默认放宽到 0.04–10m，避免远景全黑，可按场景收紧，`disparity_search_range` 控制硬件近端测距能力（848 x 530 下 64 / 128 / 256 约对应 125 / 65 / 35mm 最近距离）。Lichtblick 深度面板默认使用 Turbo Color map。算法其余参数写死在 `apps/` 中，异步日志使用 `stderr`。`rgbd_viewer <uri>` 和 `rgbd_lichtblick --camera <uri>` 可以临时覆盖结构化配置。

## 应用

```bash
rgbd_viewer --frames 100
rgbd_viewer "d435i://?width=640&height=480&fps=30" --frames 100
rgbd_replay build/session.rgbd
```

`rgbd_viewer` 支持 `--record PATH`。当前回放使用测试与回放共用的内部 `.rgbd` 格式。

### Gemini 305 + Lichtblick

`rgbd_lichtblick` 从 Gemini 305 采集 RGB-D 并实时发布到 Lichtblick WebSocket：

```bash
just gemini305-lichtblick
```

默认地址：

```text
WebSocket: ws://localhost:8765
Browser:   http://127.0.0.1:8080/?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765&layoutUrl=http%3A%2F%2F127.0.0.1%3A8080%2Frgbd-layout.json
```

Lichtblick Web 静态资源从 `data/lichtblick-web` 读取，由程序内置的 HTTP 服务托管在 `127.0.0.1:8080`。首次运行可执行：

```bash
just lichtblick-setup
```

图像话题：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

彩色图像使用 `rgb8`，深度图像使用 `mm` 单位的 `mono16`。按 `Ctrl+C` 停止。

端口来自 `config/rgbd.yaml`，也可以命令行覆盖；同时录制 MCAP：

```bash
just gemini305-lichtblick -- --port 9000 --http-port 9001 --output gemini305.mcap
```

MCAP 写入器位于 `rgbd/io`，不依赖 ROS。

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
