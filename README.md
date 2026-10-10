# facetrack

使用一个或两个摄像头运行 YOLOv8m-face，跟踪每个画面中面积最大的人脸，再从两个摄像头中选择**水平偏离画面中心最多**的目标。可显示简单 OpenCV 窗口，也可通过 ROS 2 控制颈部 Motor 1。

本仓库是独立模块，不依赖原来的 `positioning_robots` 工程，也不包含 Web 服务。

## 主要行为

- 默认尝试摄像头 `0` 和 `1`；其中一个不可用时自动退化为单摄像头。
- 每个摄像头只保留最新帧，避免推理延迟不断累积。
- 双摄像头一次批量推理；各画面先选最大人脸，再选水平偏差最大的摄像头。
- 切换摄像头需要连续确认并设置保持时间，减少左右跳动。
- 默认仅检测，不发电机命令。
- `--control` 才启用 Motor 1 控制，并发布 `/neck_control_owner` 所有权租约。
- Motor 2、Motor 3 的位置字段始终为 `NaN`，本模块不会控制它们。
- 所有摄像头画面失效超过 1 秒时，释放颈部控制权。

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
facetrack --cameras 0 1 --display --control --simulate-motor
```

真实 ROS 2 Motor 1（先阅读安全说明）：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
facetrack --cameras 0 1 --display --control
```

退出窗口按 `Q` 或 `Esc`。如果电机转向相反，增加 `--yaw-sign -1`。

## 文档

- [需求与设计](docs/REQUIREMENTS_CN.md)
- [Ubuntu + ROS 2 安装](docs/INSTALL_UBUNTU_ROS2_CN.md)
- [ROS 接口与所有权规则](docs/ROS_INTERFACE_CN.md)
- [第三方组件说明](THIRD_PARTY_NOTICES.md)

## 当前验证范围

软件单元测试、模型加载、CPU 推理和双图批量选择逻辑已经验证。尚未在目标双摄像头和真实颈部电机上做联调，因此第一次硬件测试必须限速、留急停，并确认转向和机械限位。

