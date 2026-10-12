# FaceTrack：双摄像头人脸跟踪与 ESP32 CAN3 颈部控制

这是一个可独立安装的 Ubuntu/ROS 2 模块。它使用 YOLOv8m-face 从一个或两个摄像头中检测人脸，选择水平偏离画面中心最大的目标，并可通过 ROS 2 控制 ESP32 1V2 固件的 CAN3 yaw 电机。

本仓库不依赖原来的 `positioning_robots` Web 服务。模型 `yolov8m-face.pt` 已包含在 Python 包中，运行时不需要联网下载模型。

## 功能

- 支持一个或两个摄像头；一个摄像头不可用时自动使用另一个。
- 每个摄像头只保留最新帧，避免推理积压造成越来越大的延迟。
- 双摄像头批量推理：每个画面先选择面积最大的人脸，再选择水平偏差最大的摄像头。
- 摄像头切换包含保持时间和连续确认，减少目标在两个画面之间反复跳动。
- 双摄像头预览固定显示在同一个窗口；异步取帧时保留两个面板。
- 默认只检测；只有增加 `--control` 才会发送真实电机命令。
- 订阅 `/target_selected`，在人脸跟踪和外部 Aura2/blendshape 控制之间切换。
- 使用 ESP32 1V2 的六字段 CAN 命令，只控制 CAN3。

## 支持环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- OpenCV
- Ultralytics YOLO

其他操作系统、ROS 版本和 Python 版本目前没有验证。

## 安装

先安装 ROS 2 Humble，并确认下面的文件存在：

```bash
ls /opt/ros/humble/setup.bash
```

安装系统依赖：

```bash
sudo apt update
sudo apt install -y python3.10-venv python3-pip libgl1 libglib2.0-0
```

克隆并安装：

```bash
git clone https://github.com/Lumos2024/facetrack.git
cd facetrack

chmod +x scripts/install_ubuntu.sh
./scripts/install_ubuntu.sh
source .venv/bin/activate
```

安装脚本会创建带 `--system-site-packages` 的虚拟环境，使 ROS 2 Humble 通过 apt 安装的 `rclpy` 和 `std_msgs` 可以被 Python 3.10 使用。脚本最后会自动执行测试。

## 不连接电机的测试

检查命令行：

```bash
source .venv/bin/activate
facetrack --help
```

单摄像头检测：

```bash
facetrack --cameras 0 --device cpu --display
```

双摄像头检测：

```bash
facetrack --cameras 0 1 --device cpu --display
```

如果真实摄像头编号是 `2` 和 `5`：

```bash
facetrack --cameras 2 5 --device cpu --display
```

按 `Q` 或 `Esc` 退出窗口。没有桌面环境时不要使用 `--display`。

## 使用模拟电机测试控制切换

模拟模式会订阅真正的 `/target_selected`，但不会向真实 ESP32 电机话题发布命令：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate

facetrack --cameras 0 1 --device cpu --display \
  --control --simulate-motor
```

在另一个终端测试：

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: false}'
```

## ROS 2 接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 发布 | `/neck_positions_batch_1V2` | `std_msgs/msg/Float32MultiArray` | ESP32 1V2 的六字段 CAN 命令 |
| 订阅 | `/neck_states_1V2` | `std_msgs/msg/Float32MultiArray` | 三个 CAN 电机的 24 字段反馈 |
| 订阅 | `/target_selected` | `std_msgs/msg/Bool` | FaceTrack 与外部控制的切换信号 |

ESP32 命令格式固定为：

```text
[pos1, pos2, pos3, cmd1, cmd2, cmd3]
```

当前模块只控制 CAN3：

- CAN3 位置目标：`data[2]`，单位 rad。
- CAN3 命令：`data[5]`。
- `252`：使能。
- `253`：失能。
- `254`：设置零点。
- CAN1、CAN2 位置固定填写 `0.0`。

例如 CAN3 的 `+10°` 目标：

```yaml
data: [0.0, 0.0, 0.174533, 0.0, 0.0, 0.0]
```

## `target_selected` 的准确含义

`target_selected` 是控制权切换信号，不是持续位置命令。

- 收到第一条 Bool 之前：FaceTrack 不使能、不发送 CAN3 命令。
- `false`：FaceTrack 根据人脸水平误差控制 CAN3。
- `false -> true`：停止人脸控制；等待 CAN3 反馈有效且电机已使能后，发送一次 `--selected-rad`，默认 `+30°`；随后保持静默，由外部 Aura2/blendshape 节点接管。
- 保持 `true`：FaceTrack 不继续发送 `+30°`，也不再发送人脸跟踪命令。
- `true -> false`：FaceTrack 从最新 `/neck_states_1V2` CAN3 实际位置恢复控制。

FaceTrack 会保存最后收到的 Bool，因此正常的状态切换发一次即可。但是 ROS 2 默认使用 volatile QoS：如果 Bool 在 FaceTrack 启动前已经发送，FaceTrack 收不到旧消息。外部节点应在 FaceTrack 订阅建立后至少再发布一次当前状态。

`/target_selected` 必须只有一个发布者：

```bash
ROS2CLI_NO_DAEMON=1 ros2 topic info /target_selected -v
```

当 Aura2 直接发布 `/target_selected` 时，不要同时启动另一个 bridge 重复发布 Bool。`false` 状态下，外部 blendshape 节点也必须停止发送颈部目标，否则两个控制源会互相覆盖。

## 真实 ESP32 CAN3 测试

先启动 micro-ROS Agent。下面的设备路径只是示例，请使用本机 `/dev/serial/by-id/` 中的 Espressif 路径：

```bash
source /opt/ros/humble/setup.bash
source ~/micro_ros_ws/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM1 \
  -b 921600 \
  -v 4
```

如果系统提供 `/dev/serial/by-id/` 稳定路径，应优先用该路径替换 `/dev/ttyACM1`：

```bash
ls -l /dev/serial/by-id/
```

在新终端确认 ESP32 反馈：

```bash
source /opt/ros/humble/setup.bash

ros2 topic echo --once /neck_states_1V2
ros2 topic info /neck_positions_batch_1V2 -v
```

第一次硬件测试建议限制到 `±10°`：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate

facetrack --cameras 0 1 --device cpu --display --control \
  --command-interval-s 1.0 \
  --yaw-gain 0.25 \
  --deadband 0.10 \
  --yaw-min-deg -10 \
  --yaw-max-deg 10 \
  --selected-rad 0.174533
```

确认方向、限位和急停后，可以使用当前 `±30°` 配置：

```bash
facetrack --cameras 2 5 --device cpu --display --control \
  --command-interval-s 0.2 \
  --yaw-gain 0.25 \
  --deadband 0.10 \
  --yaw-min-deg -30 \
  --yaw-max-deg 30 \
  --selected-rad 0.523599
```

收到有效 CAN3 反馈后，FaceTrack 会自动发送使能命令；正常退出时发送失能命令。如果电机方向相反，增加：

```bash
--yaw-sign -1
```

## 像素到角度的关系

人脸中心的归一化水平偏差为：

```text
dx = (face_center_x - image_width / 2) / (image_width / 2)
```

角度修正为：

```text
delta_rad = yaw_sign * dx * radians(horizontal_fov) / 2 * yaw_gain
goal_rad  = clamp(current_feedback_rad + delta_rad, yaw_min, yaw_max)
```

默认宽度 `640`、水平视场 `70°`、`yaw_gain=0.65` 时，未限幅关系约为：

```text
0.001241 rad/pixel
```

运行日志会输出 `pixel_dx`、`dx`、`raw rad` 和最终 `command rad`，可用于标定摄像头像素与真实转角的关系。

## 常见问题

### `target_selected=true`，颈部不动

依次检查：

```bash
ROS2CLI_NO_DAEMON=1 ros2 topic echo /target_selected
ROS2CLI_NO_DAEMON=1 ros2 topic info /target_selected -v
ROS2CLI_NO_DAEMON=1 ros2 topic info /neck_positions_batch_1V2 -v
ROS2CLI_NO_DAEMON=1 ros2 topic echo --once /neck_states_1V2
```

应满足：

- FaceTrack 正在运行并订阅 `/target_selected`。
- `/target_selected` 只有一个发布者。
- ESP32 在线，`/neck_positions_batch_1V2` 至少有一个订阅者。
- CAN3 反馈有效并且已经使能。
- 外部 blendshape 没有立即用另一目标覆盖一次性的 `--selected-rad`。

### 不断收到约 10 Hz 的 `false`

检查 `/target_selected` 是否存在旧的 bridge 发布者。如果 Aura2 已直接发布该话题，应停止旧 bridge，只保留一个发布者。

### Agent 显示 `Serial port not found`

先检查：

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

如果设备反复消失又出现，通常是 USB 线、Hub、供电或 ESP32 重启问题。优先使用电脑直连 USB 口、可靠的数据线和独立稳定的电机电源。

## 开发与测试

```bash
source .venv/bin/activate
python -m pytest
```

主要文件：

```text
src/neck_face_tracker/tracker.py    摄像头、YOLO、目标选择和控制循环
src/neck_face_tracker/ros_neck.py   ROS 2、ESP32 六字段协议和模拟电机
tests/                              单元测试
docs/                               安装、需求和 ROS 接口文档
```

## 安全说明

- 第一次真实测试使用 `±10°` 或更小范围。
- 保持机械急停可用，颈部周围不要有人或障碍物。
- 确认电机方向、机械零点和限位后再扩大范围。
- `/target_selected=false` 时只能由 FaceTrack 控制颈部；`true` 时只能由外部控制器接管。
- 软件限幅不能替代驱动器限位、机械限位和现场急停。

第三方模型和组件信息见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
