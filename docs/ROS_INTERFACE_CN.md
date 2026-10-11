# ROS 2 接口与切换规则

## 电机接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 发布 | `/neck_positions_batch` | `std_msgs/Float32MultiArray` | 12-float v4 电机命令 |
| 订阅 | `/neck_states` | `std_msgs/Float32MultiArray` | Motor 1 状态反馈 |
| 订阅 | `/target_selected` | `std_msgs/Bool` | 暂停/恢复人脸角度跟踪 |

Motor 1 命令使用数组中的第 1 个电机槽位。Motor 2、Motor 3 的目标位置填 `NaN`，速度和增益参数填 0。

## target_selected 行为

- `false`：根据人脸水平误差正常控制 Motor 1。
- 从 `false` 变成 `true`：取消正在运行的人脸轨迹，发送一次 `--selected-rad` 指定的位置，默认 `0.0 rad`。
- 保持 `true`：继续监听状态，但不再发布 `/neck_positions_batch`。
- 从 `true` 变回 `false`：恢复人脸跟踪。

测试命令：

```bash
ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /target_selected std_msgs/msg/Bool '{data: false}'
```

## 安全边界

`/target_selected` 是当前唯一的控制切换信号，不再使用 `/neck_control_owner`。负责发布 `/target_selected` 的节点必须保证：值为 `false` 时其他控制节点不发送颈部命令；值变成 `true` 后，先让 facetrack 发完一次固定 rad，再由其他节点接管，避免多个节点同时发布互相冲突的命令。

真实安全仍应由电机桥、机械限位、急停和现场操作流程保证。

