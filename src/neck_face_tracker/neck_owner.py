from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any


DEFAULT_OWNER_TOPIC = "/neck_control_owner"


@dataclass(frozen=True)
class NeckOwnerClaim:
    """A short-lived, advisory lease for the neck command channel."""

    owner_id: str
    active: bool
    priority: int = 50
    lease_ms: int = 500
    sent_at_s: float = 0.0

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> "NeckOwnerClaim | None":
        try:
            data = json.loads(payload)
            owner_id = str(data["owner_id"]).strip()
            if not owner_id:
                return None
            return cls(
                owner_id=owner_id,
                active=bool(data["active"]),
                priority=max(0, min(1000, int(data.get("priority", 50)))),
                lease_ms=max(100, min(10_000, int(data.get("lease_ms", 500)))),
                sent_at_s=float(data.get("sent_at_s", 0.0)),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None


class RosNeckOwnerLease:
    """Publish a renewable ROS 2 ownership flag before commanding the neck.

    The lease is deliberately independent from the motor protocol. Peers must
    implement the same rule and stop publishing neck commands while another
    owner's fresh lease is visible. This is coordination, not a hard safety
    boundary; the motor bridge remains responsible for enforcing safe limits.
    """

    def __init__(
        self,
        *,
        owner_id: str = "yolov8_face_tracker",
        topic: str = DEFAULT_OWNER_TOPIC,
        priority: int = 50,
        lease_ms: int = 500,
        heartbeat_ms: int = 100,
    ):
        cleaned_owner = str(owner_id).strip()
        if not cleaned_owner:
            raise ValueError("owner_id must not be empty")
        self.owner_id = cleaned_owner
        self.topic = str(topic).strip() or DEFAULT_OWNER_TOPIC
        self.priority = max(0, min(1000, int(priority)))
        self.lease_ms = max(100, min(10_000, int(lease_ms)))
        self.heartbeat_ms = max(50, min(self.lease_ms // 2, int(heartbeat_ms)))
        self._lock = threading.RLock()
        self._context: Any = None
        self._node: Any = None
        self._executor: Any = None
        self._publisher: Any = None
        self._message_type: Any = None
        self._spin_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False
        self._claimed = False
        self._other_claim: NeckOwnerClaim | None = None
        self._other_received_s = 0.0

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String

            context = Context()
            rclpy.init(context=context)
            suffix = re.sub(r"[^a-zA-Z0-9_]", "_", self.owner_id)[:40]
            node = rclpy.create_node(f"neck_owner_{suffix}", context=context)
            publisher = node.create_publisher(String, self.topic, 10)
            node.create_subscription(String, self.topic, self._owner_callback, 10)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
        except Exception as exc:
            raise RuntimeError(f"ROS 2 neck owner topic unavailable: {exc}") from exc

        with self._lock:
            self._context = context
            self._node = node
            self._publisher = publisher
            self._message_type = String
            self._executor = executor
            self._started = True
        self._spin_thread = threading.Thread(target=executor.spin, name="neck-owner-spin", daemon=True)
        self._spin_thread.start()

    def _owner_callback(self, message: Any) -> None:
        claim = NeckOwnerClaim.from_json(str(getattr(message, "data", "")))
        if claim is None or claim.owner_id == self.owner_id:
            return
        with self._lock:
            if claim.active:
                self._other_claim = claim
                self._other_received_s = time.monotonic()
            elif self._other_claim is not None and self._other_claim.owner_id == claim.owner_id:
                self._other_claim = None
                self._other_received_s = 0.0

    def _fresh_other_claim(self) -> NeckOwnerClaim | None:
        with self._lock:
            claim = self._other_claim
            received_s = self._other_received_s
        if claim is None:
            return None
        if time.monotonic() - received_s <= claim.lease_ms / 1000.0:
            return claim
        with self._lock:
            if self._other_claim == claim:
                self._other_claim = None
                self._other_received_s = 0.0
        return None

    def _publish(self, *, active: bool) -> None:
        with self._lock:
            publisher = self._publisher
            message_type = self._message_type
        if publisher is None or message_type is None:
            raise RuntimeError("ROS 2 neck owner publisher is not started")
        claim = NeckOwnerClaim(
            owner_id=self.owner_id,
            active=active,
            priority=self.priority,
            lease_ms=self.lease_ms,
            sent_at_s=time.time(),
        )
        message = message_type()
        message.data = claim.to_json()
        publisher.publish(message)

    def claim(self, *, settle_s: float = 0.25) -> bool:
        self.start()
        if self._fresh_other_claim() is not None:
            return False
        with self._lock:
            self._claimed = True
        self._publish(active=True)
        deadline = time.monotonic() + max(0.0, float(settle_s))
        while time.monotonic() < deadline:
            if self._fresh_other_claim() is not None:
                self.release()
                return False
            time.sleep(0.01)
        self._stop_event.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="neck-owner-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()
        return True

    def _heartbeat_loop(self) -> None:
        period_s = self.heartbeat_ms / 1000.0
        while not self._stop_event.wait(period_s):
            with self._lock:
                claimed = self._claimed
            if not claimed:
                return
            if self._fresh_other_claim() is not None:
                with self._lock:
                    self._claimed = False
                return
            try:
                self._publish(active=True)
            except Exception:
                with self._lock:
                    self._claimed = False
                return

    def owns_control(self) -> bool:
        with self._lock:
            claimed = self._claimed
        return claimed and self._fresh_other_claim() is None

    def other_owner(self) -> str | None:
        claim = self._fresh_other_claim()
        return claim.owner_id if claim is not None else None

    def release(self) -> None:
        with self._lock:
            was_claimed = self._claimed
            self._claimed = False
        self._stop_event.set()
        if was_claimed and self._publisher is not None:
            try:
                self._publish(active=False)
            except Exception:
                pass

    def close(self) -> None:
        self.release()
        heartbeat = self._heartbeat_thread
        if heartbeat is not None and heartbeat is not threading.current_thread():
            heartbeat.join(timeout=0.5)
        with self._lock:
            executor = self._executor
            node = self._node
            context = self._context
            self._started = False
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
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

