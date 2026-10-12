# FaceTrack + Aura2 互联网 Blendshape 完整启动顺序

本流程使用以下控制切换规则：

- `/target_selected=false`：FaceTrack 根据 `/neck_states_1V2` 的 CAN3 实际位置控制颈部；Aura2 必须停止颈部 blendshape 输出。
- `/target_selected=true`：FaceTrack 发送一次 `+30°`（`0.523599 rad`），然后停止发送，由 Aura2 blendshape 接管。
- FaceTrack 会保存最后收到的 Bool，因此 Aura2 发送一次 `true` 或 `false` 即可，不需要由本项目重复发布。

不要启动 `facetrack-blendshape`。这里使用原来的直接 CAN3 `facetrack`。

**不要启动 `a2f_udp_bridge`。** 当前 Aura2 已经直接发布 ROS 2 话题；再启动 bridge 会造成两个 `/target_selected` 发布者，bridge 还会以约 10 Hz 持续发送 `false`。

## 1. 终端 1：启动 micro-ROS Agent

当前电脑上的设备对应关系：

- `/dev/ttyACM0`：其他 USB 串口设备。
- `/dev/ttyACM1`：Espressif USB JTAG/Serial，即 ESP32。

启动 Agent：

```bash
source /opt/ros/humble/setup.bash
source /home/zmh/micro_ros_ws/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent \
  serial \
  --dev /dev/ttyACM1 \
  -b 921600 \
  -v 4
```

如果设备编号可能变化，可改用稳定路径：

```bash
ros2 run micro_ros_agent micro_ros_agent \
  serial \
  --dev /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_E8:F6:0A:E1:E1:12-if00 \
  -b 921600 \
  -v 4
```

`running... fd: 3` 表示 Agent 已打开设备。出现 `client_key`、`recv_message`、`send_message` 或 DDS 创建日志表示 ESP32 已连接。

## 2. 终端 2：启动 blendshape → landmark

```bash
source /home/zmh/OpenShennie_1V0/ros2_ws/env.sh

ros2 run pkg_facebot_controller blendshape2landmark --ros-args \
  -p blendshape_config_path:=/home/zmh/OpenShennie_1V0/_doc/1V2/blendshapeID_info.json \
  -p landmark_config_path:=/home/zmh/OpenShennie_1V0/_doc/1V2/landmarkID_info.json
```

该节点执行：

```text
/blendshape_robot_1V2 (61维)
→ /landmark_positions_batch_1V2 (25维)
```

## 3. 终端 3：启动 landmark → 舵机/CAN

```bash
source /home/zmh/OpenShennie_1V0/ros2_ws/env.sh

ros2 run pkg_facebot_controller landmark2angle --ros-args \
  -p landmark_config_path:=/home/zmh/OpenShennie_1V0/_doc/1V2/landmarkID_info.json
```

该节点执行：

```text
/landmark_positions_batch_1V2
→ /servo_positions_batch_1V2 (29维，最后3维是颈部 CAN1/CAN2/CAN3)
```

## 4. 终端 4：启动现有 Aura2 互联网节点

按 Aura2 项目原有的方式启动互联网接收程序。该代码不在本项目中，本项目不再启动额外 UDP bridge。

启动后检查：

```bash
source /home/zmh/OpenShennie_1V0/ros2_ws/env.sh

ROS2CLI_NO_DAEMON=1 ros2 node list | sort | grep aura2
ROS2CLI_NO_DAEMON=1 ros2 topic info /target_selected -v
ROS2CLI_NO_DAEMON=1 ros2 topic info /blendshape_robot_1V2 -v
```

当前配置应看到：

```text
/aura2_avtse_target
/aura2_a2f_woman
```

关键检查结果：

```text
/target_selected
Publisher count: 1
Node name: aura2_avtse_target

/blendshape_robot_1V2
Publisher count: 1
Node name: aura2_a2f_woman
```

如果看到 `/a2f_udp_bridge_1V2`，回到启动它的终端按 `Ctrl+C`。

## 5. 终端 5：检查状态并使能 CAN3

先确认 ESP32 状态：

```bash
source /home/zmh/OpenShennie_1V0/ros2_ws/env.sh

ROS2CLI_NO_DAEMON=1 ros2 node list
ROS2CLI_NO_DAEMON=1 ros2 topic echo --once /neck_states_1V2
```

应看到节点：

```text
/esp32_servo_1V2
```

使能 CAN3：

```bash
ros2 topic pub --once \
  /neck_positions_batch_1V2 \
  std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 252.0]}"
```

`--once` 发布成功后命令退出是正常现象。

## 6. 终端 6：启动 FaceTrack

```bash
source /home/zmh/OpenShennie_1V0/ros2_ws/env.sh
source /home/zmh/positioning_robots/.venv/bin/activate

cd /home/zmh/positioning_robots/standalone/facetrack

facetrack \
  --cameras 2 5 \
  --device cpu \
  --display \
  --control \
  --command-interval-s 0.2 \
  --yaw-gain 0.25 \
  --deadband 0.10 \
  --yaw-min-deg -30 \
  --yaw-max-deg 30 \
  --selected-rad 0.523599
```

## 运行时切换

```text
互联网 target_selected=false
→ Aura2 停止颈部 blendshape 输出
→ FaceTrack 读取最新 /neck_states_1V2 并控制 CAN3

互联网 target_selected=true
→ FaceTrack 发送一次 +30°
→ FaceTrack 静默
→ Aura2 blendshape 接管

互联网发送 target_selected=false
→ Aura2 停止颈部输出
→ FaceTrack 从最新 CAN3 实际位置恢复
```

为了减少 `true` 切换时的突跳，Aura2 的首个 HeadYaw 应从 `+0.523599 rad` 开始，再在约 1 秒内平滑过渡到网络目标角度。

## 快速检查话题

```bash
source /home/zmh/OpenShennie_1V0/ros2_ws/env.sh

ROS2CLI_NO_DAEMON=1 ros2 topic list | sort | grep -E \
  'blendshape|landmark|servo_positions|neck_states|neck_positions|target_selected'
```

预期包括：

```text
/blendshape_robot_1V2
/landmark_positions_batch_1V2
/servo_positions_batch_1V2
/neck_positions_batch_1V2
/neck_states_1V2
/target_selected
```

## 重复 `false` 排查

如果收到一次 `true` 后立即出现大量 `false`，先检查发布者：

```bash
ROS2CLI_NO_DAEMON=1 ros2 topic info /target_selected -v
```

`Publisher count` 必须是 `1`，并且只能是 `aura2_avtse_target`。如果存在 `a2f_udp_bridge_1V2`，它会用约 10 Hz 重复发送内部状态；停止该节点后再测试。
