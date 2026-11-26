# RGBD Perception

一个 **ROS-free、SDK 隔离、可测试** 的 C++17 深度相机物体感知工程骨架，目标设备为 Intel RealSense D435i 与 Orbbec Gemini 305。

当前版本提供统一数据契约、相机/检测器/深度估计接口、中心区域中位数深度融合、同步处理管线、模拟相机和单元测试。Demo 使用模拟数据，**不包含真实模型推理，也尚未接入真实相机**。

## 架构

```text
RealSense adapter ─┐
                   ├─→ FrameSet → PerceptionPipeline → Observation
Orbbec adapter ────┘                  ↑
                              IDetector + IDepthEstimator

Replay/Synthetic ──→ 同一数据契约，用于无硬件测试
```

工程约束：

- 核心层不依赖 ROS、RealSense SDK、OrbbecSDK 或推理框架。
- SDK 类型和 SDK 所有权不得越过相机适配器边界。
- 彩色图统一为 BGR；深度统一为米，NaN 表示无效值。
- `FrameSet` 自带相机 ID、时间戳、序号和对齐后的彩色内参。
- 无有效深度时显式返回质量状态，不伪造距离。
- 当前管线为同步实现；有界队列和推理调度器放在后续应用层，不藏进相机适配器。

## 环境要求

- CMake 3.20+
- 支持 C++17 的编译器（GCC、Clang 或 MSVC）
- Ninja（使用提供的 presets 时）
- Python 3.8+（Git 提交检查）
- [just](https://github.com/casey/just)（可选命令入口）
- clang-format、clang-tidy（格式和静态检查）

## 快速开始

```bash
just hooks       # 每个本地 clone 执行一次
just check       # 格式检查 + 编译 + 测试
just run         # 默认运行 5 帧模拟数据
just run --frames 10
```

不使用 just 的等价命令：

```bash
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev
./build/dev/rgbd_demo --frames 5
```

## 常用任务

```bash
just release       # Release 编译和测试
just sanitizer     # ASan + UBSan 编译和测试
just format        # 格式化第一方 C++ 代码
just format-check  # 只检查，不修改
just tidy          # clang-tidy 独立构建
just commit-check  # 检查最新提交
just install       # 安装到 build/stage
just clean
```

## 目录

```text
apps/                 可执行程序
include/rgbd/         公共数据契约与接口
src/                  核心实现
tests/                无第三方依赖的单元测试
cmake/                CMake package 配置
scripts/              Git 和开发辅助脚本
.githooks/            仓库内 Git hooks
.github/workflows/    持续集成
```

## 提交规范

采用 [Conventional Commits 1.0](https://www.conventionalcommits.org/en/v1.0.0/)：

```text
feat(camera): add Gemini 305 capture adapter
fix(perception): reject invalid depth at object edges
test(core): cover clipped detection regions
```

完整规则见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 实施路线

- [x] SDK-free 核心接口与模拟数据闭环
- [ ] RealSense D435i 适配器（可选 CMake 依赖）
- [ ] Orbbec Gemini 305 适配器（可选 CMake 依赖）
- [ ] 统一录制与回放
- [ ] ONNX Runtime 检测器
- [ ] 有界队列、双相机调度、健康监控和断线重连
- [ ] ROI 直方图与实例分割深度融合
- [ ] 可选 ROS 2 输出适配器（不进入核心层）
