# ROS 2 接口与切换规则

## 电机接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 发布 | `/neck_positions_batch_1V2` | `std_msgs/Float32MultiArray` | ESP32 1V2 的 6-float CAN 命令 |
| 订阅 | `/neck_states_1V2` | `std_msgs/Float32MultiArray` | 24-float 状态，取最后 8 个 CAN3 字段 |
| 订阅 | `/target_selected` | `std_msgs/Bool` | 暂停/恢复人脸角度跟踪 |

命令数组固定为 `[pos1,pos2,pos3,cmd1,cmd2,cmd3]`。位置单位为 rad；CAN3 yaw 目标写入 `data[2]`，CAN3 的使能/失能/归零命令写入 `data[5]`。当前硬件只使用 CAN3，CAN1、CAN2 位置填写 `0.0`。

例如 CAN3 +10°（`0.174533 rad`）：

```yaml
data: [0.0, 0.0, 0.174533, 0.0, 0.0, 0.0]
```

## target_selected 行为

- `false`：根据人脸水平误差正常控制 CAN3 yaw。
- 从 `false` 变成 `true`：取消正在运行的人脸轨迹，发送一次 `--selected-rad` 指定的位置，默认 `+30°`（`0.523599 rad`），然后让互联网连接的外部 blendshape 节点接管。
- 保持 `true`：继续监听状态，但不再发布 `/neck_positions_batch_1V2`。
- 从 `true` 变回 `false`：外部 blendshape 节点停止颈部输出；FaceTrack 读取最新 `/neck_states_1V2` CAN3 实际位置并恢复人脸跟踪。

测试命令：

```bash
ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: false}'
```

## 安全边界

`/target_selected` 是当前唯一的控制切换信号，不再使用 `/neck_control_owner`。负责发布 `/target_selected` 的节点必须保证：值为 `false` 时其他控制节点不发送颈部命令；值变成 `true` 后，先让 facetrack 发完一次固定 rad，再由其他节点接管，避免多个节点同时发布互相冲突的命令。

真实安全仍应由电机桥、机械限位、急停和现场操作流程保证。

