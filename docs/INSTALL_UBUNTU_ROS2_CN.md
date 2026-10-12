# Ubuntu 22.04 与 ROS 2 Humble 安装

这是本模块唯一支持的目标环境。即使当前没有摄像头和电机，也在 Ubuntu 22.04 上安装和执行软件测试。

## 安装

先安装基础系统包：

```bash
sudo apt update
sudo apt install -y python3.10-venv python3-pip libgl1 libglib2.0-0
```

再进入仓库根目录执行：

```bash
chmod +x scripts/install_ubuntu.sh
./scripts/install_ubuntu.sh
source .venv/bin/activate
facetrack --display
```

默认通过 udev 自动选择名称包含 `W19` 的最多两个视频采集节点，不依赖 `/dev/videoN` 编号。无桌面环境时不要传 `--display`。

有摄像头但没有电机时，可使用模拟电机测试 `/target_selected`：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
facetrack --display --control --simulate-motor --selected-rad 0.0
```

此模式订阅真实 ROS Bool topic，但不会向真实 `/neck_positions_batch_1V2` 发布电机命令。

安装脚本使用 `--system-site-packages` 创建虚拟环境，使 apt 安装的 `rclpy` 和 `std_msgs` 可以被虚拟环境中的 Python 3.10 使用。

## ROS 2 真实控制模式

先确认系统 ROS 安装有效：

```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy; from std_msgs.msg import String, Float32MultiArray"
```

然后启动现有 CAN/电机桥接节点，确认以下话题存在：

```bash
ros2 topic list
ros2 topic echo /neck_states_1V2 --once
```

首次联调使用较小范围：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
facetrack --display --control \
  --yaw-min-deg -10 --yaw-max-deg 10 --command-interval-s 1
```

收到有效反馈后，模块会发送 CAN3 yaw enable 命令；正常退出时会发送 disable 命令。不要在颈部周围有人或障碍物时开始测试。

注意：Ubuntu 22.04 的 ROS 2 Humble 使用系统 Python 3.10。创建虚拟环境时不要改用与 ROS 二进制包 ABI 不兼容的 Python 版本。

## CUDA（可选）

默认安装的 PyTorch 可能使用 CPU。需要 NVIDIA GPU 时，应先根据该电脑的驱动/CUDA 环境，从 PyTorch 官方安装说明选择对应命令，再运行本项目安装脚本。运行时可传 `--device 0`；CPU 可显式传 `--device cpu`。

