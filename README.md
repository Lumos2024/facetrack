# facetrack

使用一个或两个摄像头运行 YOLOv8m-face，跟踪每个画面中面积最大的人脸，再从两个摄像头中选择**水平偏离画面中心最多**的目标。可显示简单 OpenCV 窗口，也可通过 ROS 2 控制颈部 Motor 1。

本仓库是独立模块，不依赖原来的 `positioning_robots` 工程，也不包含 Web 服务。

## 主要行为

- 默认尝试摄像头 `0` 和 `1`；其中一个不可用时自动退化为单摄像头。
- 每个摄像头只保留最新帧，避免推理延迟不断累积。
- 双摄像头一次批量推理；各画面先选最大人脸，再选水平偏差最大的摄像头。
- 切换摄像头需要连续确认并设置保持时间，减少左右跳动。
- 默认仅检测，不发电机命令。
- `--control` 才启用 Motor 1 控制。
- 控制模式持续订阅 `/target_selected`。收到 `true` 后暂停人脸角度控制，只向电机话题发送一次固定位置（默认 `0.0 rad`）；收到 `false` 后恢复跟踪。
- Motor 2、Motor 3 的位置字段始终为 `NaN`，本模块不会控制它们。
- 所有摄像头画面失效超过 1 秒时，取消正在运行的人脸跟踪轨迹。

## 运行环境

本模块只面向 Ubuntu 22.04 + ROS 2 Humble + Python 3.10。模型 `yolov8m-face.pt` 已包含在包内，不会在运行时联网下载。

## 快速安装

```bash
./scripts/install_ubuntu.sh
source .venv/bin/activate
facetrack --cameras 0 1 --display
```

没有摄像头硬件时，可以先检查 CLI 和运行自动测试：

```bash
facetrack --help
python -m pytest
```

有一个摄像头时：

```bash
facetrack --cameras 0 --display
```

双摄像头 + 模拟电机：

```bash
source /opt/ros/humble/setup.bash
facetrack --cameras 0 1 --display --control --simulate-motor
```

模拟电机模式不会发布真实 `/neck_positions_batch`，但会订阅真正的 `/target_selected`，因此可用摄像头完整测试控制切换而不连接电机。

真实 ROS 2 Motor 1（先阅读安全说明）：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
facetrack --cameras 0 1 --display --control
```

修改 `target_selected=true` 时发送的一次性位置：

```bash
facetrack --cameras 0 1 --display --control --selected-rad 0.35
```

也可以用 `--target-selected-topic /其他话题` 修改 Bool 话题名称。保持 `true` 期间不会继续发布 `/neck_positions_batch`；`false` 后恢复人脸跟踪。

退出窗口按 `Q` 或 `Esc`。如果电机转向相反，增加 `--yaw-sign -1`。

## 文档

- [需求与设计](docs/REQUIREMENTS_CN.md)
- [Ubuntu + ROS 2 安装](docs/INSTALL_UBUNTU_ROS2_CN.md)
- [ROS 接口与切换规则](docs/ROS_INTERFACE_CN.md)
- [第三方组件说明](THIRD_PARTY_NOTICES.md)

## 当前验证范围

软件单元测试、模型加载、CPU 推理和双图批量选择逻辑已经验证。尚未在目标双摄像头和真实颈部电机上做联调，因此第一次硬件测试必须限速、留急停，并确认转向和机械限位。

