from __future__ import annotations

import argparse
import importlib
import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .ros_neck import RosNeckClient, SimulatedRosNeckClient


LOGGER = logging.getLogger("dual_camera_face_tracker")
DEFAULT_MODEL = Path(__file__).resolve().parent / "models" / "yolov8m-face.pt"


def _import_yolo_class() -> Any:
    """Load the supported Ultralytics package."""
    try:
        return importlib.import_module("ultralytics").YOLO
    except ImportError as exc:
        raise RuntimeError("Install this project first so ultralytics>=8.3 is available") from exc


@dataclass(frozen=True)
class FaceObservation:
    camera_id: str
    bbox: tuple[float, float, float, float]
    confidence: float
    frame_width: int
    frame_height: int
    captured_at_s: float

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    @property
    def normalized_area(self) -> float:
        image_area = max(1, self.frame_width * self.frame_height)
        return self.area / image_area

    @property
    def normalized_dx(self) -> float:
        center_x, _ = self.center
        half_width = max(1.0, self.frame_width / 2.0)
        return max(-1.0, min(1.0, (center_x - half_width) / half_width))


def largest_face(
    camera_id: str,
    boxes: Sequence[Sequence[float]],
    confidences: Sequence[float],
    *,
    frame_width: int,
    frame_height: int,
    captured_at_s: float,
) -> FaceObservation | None:
    """Return the largest valid face from one camera frame."""

    candidates: list[FaceObservation] = []
    for index, box in enumerate(boxes):
        if len(box) < 4:
            continue
        x1, y1, x2, y2 = (float(value) for value in box[:4])
        x1 = max(0.0, min(float(frame_width), x1))
        x2 = max(0.0, min(float(frame_width), x2))
        y1 = max(0.0, min(float(frame_height), y1))
        y2 = max(0.0, min(float(frame_height), y2))
        confidence = float(confidences[index]) if index < len(confidences) else 0.0
        observation = FaceObservation(
            camera_id=str(camera_id),
            bbox=(x1, y1, x2, y2),
            confidence=confidence,
            frame_width=int(frame_width),
            frame_height=int(frame_height),
            captured_at_s=float(captured_at_s),
        )
        if observation.area > 0.0:
            candidates.append(observation)
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item.normalized_area, item.confidence))


class MostBiasedCameraSelector:
    """Choose the camera whose largest face has the greatest horizontal bias."""

    def __init__(
        self,
        *,
        switch_margin_ratio: float = 0.20,
        min_bias_advantage: float = 0.03,
        switch_confirm_frames: int = 3,
        min_hold_s: float = 0.40,
    ):
        self.switch_margin_ratio = max(0.0, float(switch_margin_ratio))
        self.min_bias_advantage = max(0.0, float(min_bias_advantage))
        self.switch_confirm_frames = max(1, int(switch_confirm_frames))
        self.min_hold_s = max(0.0, float(min_hold_s))
        self.selected_camera: str | None = None
        self._selected_since_s = 0.0
        self._challenger_camera: str | None = None
        self._challenger_hits = 0

    def _select_now(self, camera_id: str, now_s: float) -> None:
        self.selected_camera = camera_id
        self._selected_since_s = now_s
        self._challenger_camera = None
        self._challenger_hits = 0

    def select(
        self,
        observations: Sequence[FaceObservation],
        *,
        now_s: float | None = None,
    ) -> FaceObservation | None:
        now = time.monotonic() if now_s is None else float(now_s)
        by_camera = {observation.camera_id: observation for observation in observations}
        if not by_camera:
            self._challenger_camera = None
            self._challenger_hits = 0
            return None

        if self.selected_camera not in by_camera:
            best = max(by_camera.values(), key=lambda item: (abs(item.normalized_dx), item.normalized_area))
            self._select_now(best.camera_id, now)
            return best

        current = by_camera[self.selected_camera]
        challengers = [item for camera_id, item in by_camera.items() if camera_id != self.selected_camera]
        if not challengers or now - self._selected_since_s < self.min_hold_s:
            self._challenger_camera = None
            self._challenger_hits = 0
            return current

        challenger = max(challengers, key=lambda item: (abs(item.normalized_dx), item.normalized_area))
        current_bias = abs(current.normalized_dx)
        challenger_bias = abs(challenger.normalized_dx)
        required_bias = max(
            current_bias * (1.0 + self.switch_margin_ratio),
            current_bias + self.min_bias_advantage,
        )
        if challenger_bias <= required_bias:
            self._challenger_camera = None
            self._challenger_hits = 0
            return current

        if self._challenger_camera == challenger.camera_id:
            self._challenger_hits += 1
        else:
            self._challenger_camera = challenger.camera_id
            self._challenger_hits = 1
        if self._challenger_hits >= self.switch_confirm_frames:
            self._select_now(challenger.camera_id, now)
            return challenger
        return current


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    image: Any
    sequence: int
    captured_at_s: float


class CameraWorker:
    """Keep only the newest frame so inference never builds a stale backlog."""

    def __init__(
        self,
        source: int | str,
        *,
        camera_id: str,
        width: int = 640,
        height: int = 480,
    ):
        self.source = source
        self.camera_id = camera_id
        self.width = max(160, int(width))
        self.height = max(120, int(height))
        self._lock = threading.Lock()
        self._capture: Any = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._frame: Any = None
        self._sequence = 0
        self._captured_at_s = 0.0

    def start(self) -> bool:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is required for camera capture") from exc
        backend = cv2.CAP_DSHOW if os.name == "nt" and isinstance(self.source, int) else cv2.CAP_ANY
        capture = cv2.VideoCapture(self.source, backend)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return False
        self._capture = capture
        self._thread = threading.Thread(target=self._read_loop, name=f"camera-{self.camera_id}", daemon=True)
        self._thread.start()
        return True

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            capture = self._capture
            if capture is None:
                return
            ok, frame = capture.read()
            if not ok:
                self._stop_event.wait(0.02)
                continue
            with self._lock:
                self._frame = frame
                self._sequence += 1
                self._captured_at_s = time.monotonic()

    def snapshot(self, *, max_age_s: float = 0.5) -> CameraFrame | None:
        with self._lock:
            frame = self._frame
            sequence = self._sequence
            captured_at_s = self._captured_at_s
        if frame is None or time.monotonic() - captured_at_s > max(0.05, float(max_age_s)):
            return None
        return CameraFrame(self.camera_id, frame.copy(), sequence, captured_at_s)

    def close(self) -> None:
        self._stop_event.set()
        capture = self._capture
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass
        thread = self._thread
        if thread is not None:
            thread.join(timeout=0.5)


class YoloFaceDetector:
    def __init__(
        self,
        model_path: str | Path,
        *,
        device: str = "auto",
        image_size: int = 640,
        confidence: float = 0.35,
    ):
        YOLO = _import_yolo_class()
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"YOLO face model not found: {path}")
        missing_checkpoint_dependencies = []
        for dependency in ("dill", "omegaconf"):
            try:
                importlib.import_module(dependency)
            except ImportError:
                missing_checkpoint_dependencies.append(dependency)
        if missing_checkpoint_dependencies:
            joined = " ".join(missing_checkpoint_dependencies)
            raise RuntimeError(
                f"The legacy yolov8m-face.pt checkpoint requires: {joined}. "
                f"Install the project requirements or run: pip install {joined}"
            )
        self.device = None if device.strip().lower() == "auto" else device.strip()
        self.image_size = max(320, int(image_size))
        self.confidence = max(0.0, min(1.0, float(confidence)))
        # The bundled model predates PyTorch 2.6, whose torch.load default was
        # changed to weights_only=True. This repository-owned checkpoint needs
        # the legacy loader while its pickled DetectionModel is reconstructed.
        import torch

        original_torch_load = torch.load

        def trusted_checkpoint_load(*args: Any, **kwargs: Any) -> Any:
            kwargs.setdefault("weights_only", False)
            return original_torch_load(*args, **kwargs)

        torch.load = trusted_checkpoint_load
        try:
            self.model = YOLO(str(path))
        finally:
            torch.load = original_torch_load
        try:
            self.model.fuse()
        except Exception:
            LOGGER.debug("Model fuse is not supported by this Ultralytics version", exc_info=True)

    def detect(self, frames: Sequence[CameraFrame]) -> list[FaceObservation]:
        if not frames:
            return []
        images = [item.image for item in frames]
        kwargs: dict[str, Any] = {
            "source": images,
            "classes": [0],
            "conf": self.confidence,
            "imgsz": self.image_size,
            "verbose": False,
        }
        if self.device is not None:
            kwargs["device"] = self.device
        results = self.model.predict(**kwargs)
        observations: list[FaceObservation] = []
        for frame, result in zip(frames, results):
            boxes_object = getattr(result, "boxes", None)
            if boxes_object is None or len(boxes_object) == 0:
                continue
            boxes = boxes_object.xyxy.detach().cpu().numpy().tolist()
            confidence_tensor = getattr(boxes_object, "conf", None)
            confidences = (
                confidence_tensor.detach().cpu().numpy().tolist()
                if confidence_tensor is not None
                else [0.0] * len(boxes)
            )
            height, width = frame.image.shape[:2]
            observation = largest_face(
                frame.camera_id,
                boxes,
                confidences,
                frame_width=width,
                frame_height=height,
                captured_at_s=frame.captured_at_s,
            )
            if observation is not None:
                observations.append(observation)
        return observations


class FaceYawController:
    def __init__(
        self,
        neck: RosNeckClient,
        *,
        deadband: float = 0.10,
        horizontal_fov_deg: float = 70.0,
        gain: float = 0.65,
        yaw_sign: int = 1,
        max_step_deg: float = 5.0,
        yaw_min_deg: float = -45.0,
        yaw_max_deg: float = 45.0,
        min_command_interval_s: float = 0.20,
        speed_deg_s: float = 25.0,
    ):
        self.neck = neck
        self.deadband = max(0.0, min(0.5, float(deadband)))
        self.horizontal_fov_deg = max(10.0, min(180.0, float(horizontal_fov_deg)))
        self.gain = max(0.05, min(2.0, float(gain)))
        self.yaw_sign = -1 if int(yaw_sign) < 0 else 1
        self.max_step_deg = max(0.1, min(30.0, float(max_step_deg)))
        self.yaw_min_deg = float(min(yaw_min_deg, yaw_max_deg))
        self.yaw_max_deg = float(max(yaw_min_deg, yaw_max_deg))
        self.min_command_interval_s = max(0.05, float(min_command_interval_s))
        self.speed_deg_s = max(1.0, float(speed_deg_s))
        self._last_command_s = 0.0

    def update(self, target: FaceObservation, *, now_s: float | None = None) -> dict[str, float] | None:
        now = time.monotonic() if now_s is None else float(now_s)
        if abs(target.normalized_dx) <= self.deadband:
            return None
        if now - self._last_command_s < self.min_command_interval_s:
            return None
        status = self.neck.status()
        if not status.connected or not status.enabled or status.position_deg is None or status.moving:
            return None
        raw_delta = (
            self.yaw_sign
            * target.normalized_dx
            * (self.horizontal_fov_deg / 2.0)
            * self.gain
        )
        delta_deg = max(-self.max_step_deg, min(self.max_step_deg, raw_delta))
        goal_deg = max(self.yaw_min_deg, min(self.yaw_max_deg, status.position_deg + delta_deg))
        effective_delta = goal_deg - status.position_deg
        if abs(effective_delta) < 0.05:
            return None
        duration_s = max(0.28, abs(effective_delta) / self.speed_deg_s)
        motion = self.neck.move_motor1(goal_deg, duration_s)
        self._last_command_s = now
        return motion


def _parse_camera_source(value: str) -> int | str:
    stripped = str(value).strip()
    return int(stripped) if stripped.isdigit() else stripped


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Track the largest YOLOv8 face using one or two neck cameras")
    parser.add_argument("--cameras", nargs="+", default=["0", "1"], help="One or two camera indexes or video paths")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="YOLOv8-face .pt model")
    parser.add_argument("--device", default="auto", help="Ultralytics device, for example auto, cpu, 0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--deadband", type=float, default=0.10, help="Normalized horizontal deadband")
    parser.add_argument("--horizontal-fov", type=float, default=70.0)
    parser.add_argument("--yaw-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--max-step-deg", type=float, default=5.0)
    parser.add_argument("--yaw-min-deg", type=float, default=-45.0)
    parser.add_argument("--yaw-max-deg", type=float, default=45.0)
    parser.add_argument("--control", action="store_true", help="Actually command ROS Motor 1")
    parser.add_argument("--simulate-motor", action="store_true", help="Use the in-process simulated Motor 1")
    parser.add_argument(
        "--target-selected-topic",
        default="/target_selected",
        help="std_msgs/Bool topic that pauses face tracking and triggers one fixed-rad command",
    )
    parser.add_argument(
        "--selected-rad",
        type=float,
        default=0.0,
        help="Motor 1 position sent once when target_selected becomes true",
    )
    parser.add_argument("--display", action="store_true", help="Show camera windows; requires non-headless OpenCV")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    return parser


def _draw_frame(frame: CameraFrame, observations: Sequence[FaceObservation], selected: FaceObservation | None) -> Any:
    import cv2

    image = frame.image.copy()
    cv2.putText(
        image,
        f"CAMERA {frame.camera_id}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (255, 255, 255),
        2,
    )
    observation = next((item for item in observations if item.camera_id == frame.camera_id), None)
    if frame.sequence == 0:
        cv2.putText(
            image,
            "Waiting for frame...",
            (10, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )
    elif observation is not None:
        x1, y1, x2, y2 = (int(value) for value in observation.bbox)
        color = (0, 0, 255) if selected is not None and selected.camera_id == frame.camera_id else (0, 255, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cx, cy = (int(value) for value in observation.center)
        cv2.circle(image, (cx, cy), 4, color, -1)
        cv2.putText(
            image,
            f"dx={observation.normalized_dx:+.3f} area={observation.normalized_area:.3f}",
            (10, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
        )
    else:
        cv2.putText(image, "No face", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    return image


def _compose_display(
    frames: Sequence[CameraFrame],
    observations: Sequence[FaceObservation],
    selected: FaceObservation | None,
    *,
    status_text: str,
) -> Any:
    import cv2

    rendered = [_draw_frame(frame, observations, selected) for frame in frames]
    target_height = min(image.shape[0] for image in rendered)
    normalized = []
    for image in rendered:
        if image.shape[0] != target_height:
            scale = target_height / image.shape[0]
            image = cv2.resize(image, (max(1, round(image.shape[1] * scale)), target_height))
        normalized.append(image)
    canvas = normalized[0] if len(normalized) == 1 else cv2.hconcat(normalized)
    cv2.rectangle(canvas, (0, max(0, canvas.shape[0] - 34)), (canvas.shape[1], canvas.shape[0]), (0, 0, 0), -1)
    cv2.putText(
        canvas,
        f"{status_text} | Q/Esc: quit",
        (10, canvas.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        1,
    )
    return canvas


def _update_display_frame_cache(
    cache: dict[str, CameraFrame],
    fresh_frames: Sequence[CameraFrame],
    camera_order: Sequence[str],
    *,
    placeholder_size: tuple[int, int] | None = None,
) -> list[CameraFrame]:
    """Keep both camera panels visible when their frames arrive asynchronously."""
    for frame in fresh_frames:
        cache[frame.camera_id] = frame
    displayed: list[CameraFrame] = []
    for camera_id in camera_order:
        frame = cache.get(camera_id)
        if frame is None and placeholder_size is not None:
            import numpy as np

            width, height = placeholder_size
            frame = CameraFrame(
                camera_id,
                np.zeros((max(1, height), max(1, width), 3), dtype=np.uint8),
                0,
                0.0,
            )
        if frame is not None:
            displayed.append(frame)
    return displayed


def run(args: argparse.Namespace) -> int:
    if len(args.cameras) > 2:
        raise ValueError("This tracker supports at most two cameras")
    workers: list[CameraWorker] = []
    neck: RosNeckClient | None = None
    try:
        for index, value in enumerate(args.cameras):
            source = _parse_camera_source(value)
            worker = CameraWorker(source, camera_id=str(index), width=args.width, height=args.height)
            if worker.start():
                workers.append(worker)
                LOGGER.info("Camera %s opened from %r", worker.camera_id, source)
            else:
                LOGGER.warning("Camera %s unavailable from %r", worker.camera_id, source)
        if not workers:
            LOGGER.error("No usable camera; tracker cannot start")
            return 2
        LOGGER.info("Tracking with %d camera(s)", len(workers))

        detector = YoloFaceDetector(
            args.model,
            device=args.device,
            image_size=args.imgsz,
            confidence=args.confidence,
        )
        selector = MostBiasedCameraSelector()
        controller: FaceYawController | None = None
        motor_enable_requested = False
        if args.control:
            if args.simulate_motor:
                neck = SimulatedRosNeckClient(
                    target_selected_topic=args.target_selected_topic,
                    listen_to_ros=True,
                )
                neck.enable_motor1()
                if not neck.target_selected_listener_available():
                    LOGGER.warning(
                        "ROS 2 target listener unavailable; source ROS 2 before testing %s",
                        args.target_selected_topic,
                    )
            else:
                neck = RosNeckClient(target_selected_topic=args.target_selected_topic)
            controller = FaceYawController(
                neck,
                deadband=args.deadband,
                horizontal_fov_deg=args.horizontal_fov,
                yaw_sign=args.yaw_sign,
                max_step_deg=args.max_step_deg,
                yaw_min_deg=args.yaw_min_deg,
                yaw_max_deg=args.yaw_max_deg,
            )
            LOGGER.info("Motor control enabled%s", " (simulated)" if args.simulate_motor else "")
        else:
            LOGGER.info("Detection-only mode; pass --control to command Motor 1")

        previous_sequences: dict[str, int] = {}
        display_frame_cache: dict[str, CameraFrame] = {}
        camera_order = [worker.camera_id for worker in workers]
        selected_camera: str | None = None
        last_live_frame_s = time.monotonic()
        camera_loss_motion_cancelled = False
        target_selected_was_active = False
        selected_rad_sent = False
        while True:
            frames = [frame for worker in workers if (frame := worker.snapshot()) is not None]
            now_s = time.monotonic()
            target_selected_active = bool(neck is not None and neck.target_selected())
            if target_selected_active != target_selected_was_active:
                if target_selected_active:
                    neck.cancel_motor1_motion()
                    LOGGER.info(
                        "%s=true: face-angle control paused; waiting to send %.6f rad once",
                        args.target_selected_topic,
                        args.selected_rad,
                    )
                else:
                    selected_rad_sent = False
                    LOGGER.info("%s=false: face tracking resumed", args.target_selected_topic)
                target_selected_was_active = target_selected_active

            if neck is not None and not args.simulate_motor and not motor_enable_requested:
                motor_status = neck.status()
                if motor_status.connected:
                    if motor_status.enabled:
                        LOGGER.info("Motor 1 is already enabled")
                    else:
                        neck.enable_motor1()
                        LOGGER.info("Motor 1 enable command sent after valid feedback")
                    motor_enable_requested = True

            if target_selected_active and neck is not None and not selected_rad_sent:
                motor_status = neck.status()
                if motor_status.connected and motor_status.enabled:
                    command = neck.send_motor1_position_once(args.selected_rad)
                    selected_rad_sent = True
                    LOGGER.info(
                        "%s=true: sent Motor 1 target %.6f rad (%.3f deg) once; motor topic now silent",
                        args.target_selected_topic,
                        command["goal_rad"],
                        command["goal_deg"],
                    )
            if not frames:
                if (
                    neck is not None
                    and not target_selected_active
                    and not camera_loss_motion_cancelled
                    and now_s - last_live_frame_s >= 1.0
                ):
                    neck.cancel_motor1_motion()
                    camera_loss_motion_cancelled = True
                    LOGGER.warning("All camera streams are stale; face-tracking motion cancelled")
                time.sleep(0.02)
                continue
            last_live_frame_s = time.monotonic()
            camera_loss_motion_cancelled = False
            sequences = {frame.camera_id: frame.sequence for frame in frames}
            if sequences == previous_sequences:
                time.sleep(0.005)
                continue
            previous_sequences = sequences
            observations = [] if target_selected_active else detector.detect(frames)
            selected = None if target_selected_active else selector.select(observations)
            if selected is None:
                LOGGER.debug("No face in active camera frames")
            else:
                if selected.camera_id != selected_camera:
                    selected_camera = selected.camera_id
                    LOGGER.info(
                        "Selected camera %s: dx=%+.3f area=%.3f",
                        selected.camera_id,
                        selected.normalized_dx,
                        selected.normalized_area,
                    )
                if controller is not None and not target_selected_active:
                    motion = controller.update(selected)
                    if motion is not None:
                        LOGGER.info(
                            "Motor goal %.2f deg from camera %s (dx=%+.3f)",
                            motion["goal_deg"],
                            selected.camera_id,
                            selected.normalized_dx,
                        )

            if args.display:
                try:
                    import cv2

                    if neck is not None:
                        neck_status = neck.status()
                        motor_text = (
                            f"motor={neck_status.position_deg:+.1f}deg"
                            if neck_status.position_deg is not None
                            else "motor=no-feedback"
                        )
                    else:
                        motor_text = "detection-only"
                    selected_text = (
                        "target_selected=true"
                        if target_selected_active
                        else (f"selected={selected.camera_id}" if selected is not None else "selected=none")
                    )
                    display_frames = _update_display_frame_cache(
                        display_frame_cache,
                        frames,
                        camera_order,
                        placeholder_size=(args.width, args.height),
                    )
                    canvas = _compose_display(
                        display_frames,
                        observations,
                        selected,
                        status_text=f"{selected_text} | {motor_text}",
                    )
                    cv2.imshow("YOLOv8m Face Tracker", canvas)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                except cv2.error as exc:
                    raise RuntimeError("--display needs opencv-python, not opencv-python-headless") from exc
    except KeyboardInterrupt:
        LOGGER.info("Stopped by user")
    finally:
        for worker in workers:
            worker.close()
        if neck is not None:
            neck.close()
        if args.display:
            try:
                import cv2

                cv2.destroyAllWindows()
            except Exception:
                pass
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())

