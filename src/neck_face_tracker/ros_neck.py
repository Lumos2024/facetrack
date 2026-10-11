from __future__ import annotations

import math
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any


MOTOR_COUNT = 3
STATE_FIELDS = 8
CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_SET_ZERO = 0xFE
PUBLISH_PERIOD_S = 0.01
MOTOR1_HOLD_KP = 30.0
MOTOR1_HOLD_KD = 1.0
MOTOR1_MOVE_KP = 40.0
MOTOR1_MOVE_KD = 1.0
YAW_LIMIT_DEG = 180.0


def neck_command_payload(command: int) -> list[float]:
    """Build a v4 command that targets Motor 1 and ignores Motors 2/3."""

    return [math.nan] * 3 + [0.0] * 3 + [float(command), 0.0, 0.0] + [0.0] * 3


def neck_motion_payload(position: float, velocity: float, blend: float) -> list[float]:
    """Build a v4 trajectory sample for Motor 1 only."""

    return [float(position), math.nan, math.nan, float(velocity), 0.0, 0.0, 0.0, 0.0, 0.0, float(blend), 0.0, 0.0]


@dataclass(frozen=True)
class NeckYawStatus:
    available: bool = False
    connected: bool = False
    valid: bool = False
    enabled: bool = False
    position_rad: float | None = None
    position_deg: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    driver_temp_c: float | None = None
    motor_temp_c: float | None = None
    error_code: int | None = None
    feedback_age_s: float | None = None
    command_subscribers: int = 0
    other_command_publishers: tuple[str, ...] = ()
    moving: bool = False
    gain_blend: float = 0.0
    kp: float = MOTOR1_HOLD_KP
    kd: float = MOTOR1_HOLD_KD
    message: str = "ROS 2 not started"


class RosNeckClient:
    """ROS 2 client for CAN Motor 1 yaw commissioning.

    The client publishes the 12-float neck v4 protocol. Positions for Motors 2
    and 3 are always NaN, so this project cannot change either motor.
    """

    def __init__(
        self,
        *,
        auto_start: bool = True,
        target_selected_topic: str = "/target_selected",
        motor_io: bool = True,
    ):
        self._lock = threading.RLock()
        self._available = False
        self._message = "ROS 2 not started"
        self._state: list[float] | None = None
        self._last_feedback_s = 0.0
        self._publisher: Any = None
        self._message_type: Any = None
        self._context: Any = None
        self._node: Any = None
        self._executor: Any = None
        self._spin_thread: threading.Thread | None = None
        self._trajectory_thread: threading.Thread | None = None
        self._trajectory_generation = 0
        self._moving = False
        self._gain_blend = 0.0
        self._other_publishers_cache: tuple[str, ...] = ()
        self._other_publishers_checked_s = 0.0
        self._commanded_enable = False
        self._target_selected_topic = str(target_selected_topic).strip() or "/target_selected"
        self._target_selected = False
        self._motor_io = bool(motor_io)
        if auto_start:
            self.start()

    def start(self) -> None:
        with self._lock:
            if self._available:
                return
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import Bool, Float32MultiArray

            context = Context()
            rclpy.init(context=context)
            node_name = "positioning_neck_yaw" if self._motor_io else "facetrack_simulated_neck"
            node = rclpy.create_node(node_name, context=context)
            publisher = None
            message_type = None
            if self._motor_io:
                publisher = node.create_publisher(Float32MultiArray, "neck_positions_batch", 10)
                node.create_subscription(Float32MultiArray, "neck_states", self._state_callback, 10)
                message_type = Float32MultiArray
            node.create_subscription(Bool, self._target_selected_topic, self._target_selected_callback, 10)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
        except Exception as exc:
            with self._lock:
                self._message = f"ROS 2 unavailable: {exc}"
            return

        with self._lock:
            self._context = context
            self._node = node
            self._publisher = publisher
            self._message_type = message_type
            self._executor = executor
            self._available = True
            self._message = "waiting for /neck_states" if self._motor_io else "simulated target listener ready"
        self._spin_thread = threading.Thread(target=executor.spin, name="neck-ros-spin", daemon=True)
        self._spin_thread.start()

    def _state_callback(self, message: Any) -> None:
        self.update_state(list(message.data))

    def _target_selected_callback(self, message: Any) -> None:
        with self._lock:
            self._target_selected = bool(message.data)

    def target_selected(self) -> bool:
        with self._lock:
            return self._target_selected

    def target_selected_listener_available(self) -> bool:
        with self._lock:
            return self._node is not None

    def update_state(self, values: list[float]) -> None:
        if len(values) < STATE_FIELDS:
            return
        with self._lock:
            self._state = [float(value) for value in values[:STATE_FIELDS]]
            self._last_feedback_s = time.monotonic()
            self._message = "Motor 1 feedback valid" if values[7] > 0.5 else "Motor 1 feedback invalid"

    def status(self) -> NeckYawStatus:
        now = time.monotonic()
        with self._lock:
            state = list(self._state) if self._state is not None else None
            age = now - self._last_feedback_s if self._last_feedback_s else None
            available = self._available
            moving = self._moving
            message = self._message
            publisher = self._publisher
            gain_blend = self._gain_blend
        subscribers = 0
        if publisher is not None:
            try:
                subscribers = int(publisher.get_subscription_count())
            except Exception:
                subscribers = 0
        other_publishers = self._other_command_publishers()
        valid = bool(state is not None and state[7] > 0.5)
        connected = bool(available and valid and age is not None and age <= 1.5)
        if available and not connected and age is not None and age > 1.5:
            message = "Motor 1 feedback timeout"
        return NeckYawStatus(
            available=available,
            connected=connected,
            valid=valid,
            enabled=bool(state is not None and state[6] > 0.5),
            position_rad=state[0] if valid else None,
            position_deg=math.degrees(state[0]) if valid else None,
            velocity_rad_s=state[1] if valid else None,
            torque_nm=state[2] if valid else None,
            driver_temp_c=state[3] if valid else None,
            motor_temp_c=state[4] if valid else None,
            error_code=int(state[5]) if valid else None,
            feedback_age_s=age,
            command_subscribers=subscribers,
            other_command_publishers=other_publishers,
            moving=moving,
            gain_blend=gain_blend,
            kp=MOTOR1_HOLD_KP + (MOTOR1_MOVE_KP - MOTOR1_HOLD_KP) * gain_blend,
            kd=MOTOR1_HOLD_KD + (MOTOR1_MOVE_KD - MOTOR1_HOLD_KD) * gain_blend,
            message=message,
        )

    def _other_command_publishers(self, *, force: bool = False) -> tuple[str, ...]:
        now = time.monotonic()
        with self._lock:
            node = self._node
            cached = self._other_publishers_cache
            checked_s = self._other_publishers_checked_s
        if node is None:
            return ()
        if not force and now - checked_s < 1.0:
            return cached
        try:
            own_name = node.get_name()
            names = []
            for info in node.get_publishers_info_by_topic("/neck_positions_batch"):
                if info.node_name == own_name:
                    continue
                namespace = str(info.node_namespace or "/").rstrip("/")
                names.append(f"{namespace}/{info.node_name}" if namespace else f"/{info.node_name}")
            result = tuple(sorted(set(names)))
            with self._lock:
                self._other_publishers_cache = result
                self._other_publishers_checked_s = now
            return result
        except Exception:
            return cached

    def status_dict(self) -> dict[str, Any]:
        result = asdict(self.status())
        for key in ("position_rad", "position_deg", "velocity_rad_s", "torque_nm", "feedback_age_s"):
            if result[key] is not None:
                result[key] = round(float(result[key]), 4)
        result["motor_id"] = 1
        result["topic_command"] = "/neck_positions_batch"
        result["topic_state"] = "/neck_states"
        result["yaw_limit_deg"] = YAW_LIMIT_DEG
        result["publish_hz"] = round(1.0 / PUBLISH_PERIOD_S)
        result["hold_kp"] = MOTOR1_HOLD_KP
        result["hold_kd"] = MOTOR1_HOLD_KD
        result["move_kp"] = MOTOR1_MOVE_KP
        result["move_kd"] = MOTOR1_MOVE_KD
        return result

    def _publish(self, values: list[float]) -> None:
        with self._lock:
            publisher = self._publisher
            message_type = self._message_type
        if publisher is None or message_type is None:
            raise RuntimeError(self.status().message)
        message = message_type()
        message.data = values
        publisher.publish(message)

    def enable_motor1(self) -> NeckYawStatus:
        status = self.status()
        if not status.connected:
            raise RuntimeError("Motor 1 has no valid /neck_states feedback")
        if status.error_code not in (None, 0, 1):
            raise RuntimeError(f"Motor 1 error code is {status.error_code}")
        self._publish(neck_command_payload(CMD_ENABLE))
        with self._lock:
            self._commanded_enable = True
            self._message = "Motor 1 enable command sent"
        return self.status()

    def disable_motor1(self) -> NeckYawStatus:
        self._cancel_trajectory(wait=True)
        self._publish(neck_command_payload(CMD_DISABLE))
        with self._lock:
            self._commanded_enable = False
            self._message = "Motor 1 disable command sent"
        return self.status()

    def hold_motor1(self) -> NeckYawStatus:
        self._cancel_trajectory(wait=True)
        status = self.status()
        if not status.connected or not status.enabled or status.position_rad is None:
            raise RuntimeError("Motor 1 must have valid feedback and be enabled")
        self._publish(neck_motion_payload(status.position_rad, 0.0, 0.0))
        with self._lock:
            self._gain_blend = 0.0
            self._message = "Motor 1 holding current position"
        return self.status()

    def set_zero_motor1(self) -> NeckYawStatus:
        self._cancel_trajectory(wait=True)
        status = self.status()
        if not status.connected:
            raise RuntimeError("Motor 1 has no valid /neck_states feedback")
        self._publish(neck_command_payload(CMD_SET_ZERO))
        with self._lock:
            self._message = "Motor 1 set-zero command sent"
        return self.status()

    def move_motor1(self, target_deg: float, duration_s: float) -> dict[str, Any]:
        status = self.status()
        if not status.connected or status.position_rad is None:
            raise RuntimeError("Motor 1 has no valid /neck_states feedback")
        if not status.enabled:
            raise RuntimeError("Enable Motor 1 before moving")
        if status.error_code not in (0, 1):
            raise RuntimeError(f"Motor 1 error code is {status.error_code}")
        goal_deg = max(-YAW_LIMIT_DEG, min(YAW_LIMIT_DEG, float(target_deg)))
        duration = max(0.2, min(30.0, float(duration_s)))
        start = status.position_rad
        goal = math.radians(goal_deg)
        self._start_trajectory(start, goal, duration)
        return {
            "start_deg": round(math.degrees(start), 3),
            "goal_deg": round(goal_deg, 3),
            "delta_deg": round(goal_deg - math.degrees(start), 3),
            "duration_s": duration,
        }

    def jog_motor1(self, delta_deg: float, duration_s: float) -> dict[str, Any]:
        status = self.status()
        if not status.connected or status.position_rad is None:
            raise RuntimeError("Motor 1 has no valid /neck_states feedback")
        delta = max(-10.0, min(10.0, float(delta_deg)))
        goal_deg = max(-YAW_LIMIT_DEG, min(YAW_LIMIT_DEG, math.degrees(status.position_rad) + delta))
        return self.move_motor1(goal_deg, duration_s)

    def cancel_motor1_motion(self) -> None:
        """Cancel a running face-tracking trajectory without publishing."""

        self._cancel_trajectory(wait=True)

    def send_motor1_position_once(self, target_rad: float) -> dict[str, Any]:
        """Publish one Motor 1 position command, then leave the topic silent."""

        self._cancel_trajectory(wait=True)
        status = self.status()
        if not status.connected or status.position_rad is None:
            raise RuntimeError("Motor 1 has no valid /neck_states feedback")
        if not status.enabled:
            raise RuntimeError("Enable Motor 1 before sending a selected target")
        if status.error_code not in (0, 1):
            raise RuntimeError(f"Motor 1 error code is {status.error_code}")
        if not math.isfinite(float(target_rad)):
            raise ValueError("Selected target rad must be finite")
        limit_rad = math.radians(YAW_LIMIT_DEG)
        goal_rad = max(-limit_rad, min(limit_rad, float(target_rad)))
        self._publish(neck_motion_payload(goal_rad, 0.0, 0.0))
        with self._lock:
            self._gain_blend = 0.0
            self._message = "Motor 1 selected target sent once"
        return {
            "start_rad": round(status.position_rad, 6),
            "goal_rad": round(goal_rad, 6),
            "goal_deg": round(math.degrees(goal_rad), 3),
        }

    @staticmethod
    def _quintic(progress: float) -> tuple[float, float, float]:
        t = max(0.0, min(1.0, progress))
        position_scale = t**3 * (10.0 + t * (-15.0 + 6.0 * t))
        velocity_scale = 30.0 * t * t * (1.0 - t) ** 2
        ramp = 0.2
        smooth_in = min(1.0, t / ramp)
        smooth_out = min(1.0, (1.0 - t) / ramp)
        smooth_in = smooth_in * smooth_in * (3.0 - 2.0 * smooth_in)
        smooth_out = smooth_out * smooth_out * (3.0 - 2.0 * smooth_out)
        return position_scale, velocity_scale, min(smooth_in, smooth_out)

    def _start_trajectory(self, start: float, goal: float, duration: float) -> None:
        with self._lock:
            self._trajectory_generation += 1
            generation = self._trajectory_generation
            self._moving = True
            self._message = "Motor 1 trajectory running"

        def run() -> None:
            started = time.monotonic()
            next_tick = started
            while True:
                with self._lock:
                    if generation != self._trajectory_generation:
                        return
                    state = list(self._state) if self._state is not None else None
                    feedback_age = time.monotonic() - self._last_feedback_s if self._last_feedback_s else math.inf
                feedback_ready = bool(
                    state is not None
                    and len(state) >= STATE_FIELDS
                    and state[7] > 0.5
                    and state[6] > 0.5
                    and int(state[5]) in (0, 1)
                    and feedback_age <= 1.5
                )
                if not feedback_ready:
                    with self._lock:
                        if generation == self._trajectory_generation:
                            self._moving = False
                            self._gain_blend = 0.0
                            self._message = "trajectory stopped: Motor 1 feedback lost or disabled"
                    return
                elapsed = time.monotonic() - started
                progress = min(1.0, elapsed / duration)
                position_scale, velocity_scale, blend = self._quintic(progress)
                position = start + (goal - start) * position_scale
                velocity = (goal - start) * velocity_scale / duration
                try:
                    self._publish(neck_motion_payload(position, velocity, blend))
                    with self._lock:
                        self._gain_blend = blend
                except Exception as exc:
                    with self._lock:
                        self._moving = False
                        self._message = f"trajectory failed: {exc}"
                    return
                if progress >= 1.0:
                    with self._lock:
                        if generation == self._trajectory_generation:
                            self._moving = False
                            self._gain_blend = 0.0
                            self._message = "Motor 1 trajectory complete"
                    return
                next_tick += PUBLISH_PERIOD_S
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                elif delay < -PUBLISH_PERIOD_S:
                    # Do not queue stale trajectory samples after CPU stalls.
                    next_tick = time.monotonic()

        self._trajectory_thread = threading.Thread(target=run, name="neck-yaw-trajectory", daemon=True)
        self._trajectory_thread.start()

    def _cancel_trajectory(self, *, wait: bool = False) -> None:
        with self._lock:
            self._trajectory_generation += 1
            self._moving = False
            self._gain_blend = 0.0
            thread = self._trajectory_thread
        if wait and thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.2)

    def close(self) -> None:
        self._cancel_trajectory(wait=True)
        with self._lock:
            should_disable = self._commanded_enable and self._publisher is not None
        if should_disable:
            try:
                self._publish(neck_command_payload(CMD_DISABLE))
                time.sleep(0.03)
            except Exception:
                pass
        with self._lock:
            executor = self._executor
            node = self._node
            context = self._context
            spin_thread = self._spin_thread
            self._available = False
            self._commanded_enable = False
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        if spin_thread is not None and spin_thread is not threading.current_thread():
            spin_thread.join(timeout=1.0)
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            if context is not None and context.ok():
                context.shutdown()
        except Exception:
            pass


class SimulatedRosNeckClient(RosNeckClient):
    def __init__(
        self,
        *,
        target_selected_topic: str = "/target_selected",
        listen_to_ros: bool = False,
    ):
        super().__init__(
            auto_start=listen_to_ros,
            target_selected_topic=target_selected_topic,
            motor_io=False,
        )
        self._available = True
        self._message = "simulated Motor 1 feedback"
        self.update_state([0.0, 0.0, 0.0, 25.0, 25.0, 0.0, 0.0, 1.0])

    def status(self) -> NeckYawStatus:
        with self._lock:
            self._last_feedback_s = time.monotonic()
        return super().status()

    def _publish(self, values: list[float]) -> None:
        command = int(values[6]) if len(values) >= 9 and math.isfinite(values[6]) else 0
        with self._lock:
            state = list(self._state or [0.0] * STATE_FIELDS)
            if command == CMD_ENABLE:
                state[6] = 1.0
            elif command == CMD_DISABLE:
                state[6] = 0.0
            if values and math.isfinite(values[0]):
                state[0] = float(values[0])
                state[1] = float(values[3])
            state[7] = 1.0
        self.update_state(state)

    def close(self) -> None:
        super().close()
