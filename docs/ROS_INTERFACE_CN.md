# ROS 2 接口与所有权规则

## 电机接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 发布 | `/neck_positions_batch` | `std_msgs/Float32MultiArray` | 12-float v4 电机命令 |
| 订阅 | `/neck_states` | `std_msgs/Float32MultiArray` | Motor 1 状态反馈 |
| 发布/订阅 | `/neck_control_owner` | `std_msgs/String` | JSON 所有权短租约 |

Motor 1 命令使用数组中的第 1 个电机槽位。Motor 2、Motor 3 的目标位置填 `NaN`，速度和增益参数填 0。

## owner 消息

示例：

```json
{"active":true,"lease_ms":500,"owner_id":"yolov8_face_tracker","priority":50,"sent_at_s":1760000000.0}
```

字段含义：

- `owner_id`：控制节点的唯一名称。
- `active`：`true` 表示取得控制，`false` 表示主动释放。
- `lease_ms`：租约有效期，默认 500 ms。
- `sent_at_s`：发送端 Unix 时间，仅用于日志；接收端用本地单调时钟判断过期。
- `priority`：保留的协调优先级字段。当前版本遇到其他有效 owner 时直接让出，不做抢占。

控制节点约每 100 ms 续租。其他颈部控制节点如果采用此协议，应在看到另一个未过期的 owner 后立即停止发布 `/neck_positions_batch`。

## 安全边界

owner 是协作信号，无法阻止不遵守协议的节点发布。因此本模块还会检查 `/neck_positions_batch` 的其他 publisher；默认检测到其他 publisher 就拒绝动作。只有确认全部节点都正确服从 owner 协议后，才允许传入 `--allow-coordinated-publishers`。

真实安全仍应由电机桥、机械限位、急停和现场操作流程保证。

