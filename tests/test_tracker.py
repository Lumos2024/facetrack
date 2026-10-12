import unittest
from pathlib import Path

import numpy as np

from neck_face_tracker.tracker import (
    CameraFrame,
    FaceObservation,
    FaceYawController,
    MostBiasedCameraSelector,
    _compose_display,
    _update_display_frame_cache,
    discover_matching_cameras,
    largest_face,
)
from neck_face_tracker.ros_neck import SimulatedRosNeckClient


def observation(camera_id: str, normalized_dx: float, *, area_ratio: float = 0.1) -> FaceObservation:
    width = 640
    height = 480
    box_width = (area_ratio * width * height) ** 0.5
    center_x = width / 2.0 + normalized_dx * width / 2.0
    return FaceObservation(
        camera_id=camera_id,
        bbox=(center_x - box_width / 2.0, 100.0, center_x + box_width / 2.0, 100.0 + box_width),
        confidence=0.9,
        frame_width=width,
        frame_height=height,
        captured_at_s=1.0,
    )


class LargestFaceTests(unittest.TestCase):
    def test_largest_face_is_selected_within_each_camera(self):
        selected = largest_face(
            "left",
            [(10, 10, 60, 60), (100, 100, 300, 350)],
            [0.99, 0.75],
            frame_width=640,
            frame_height=480,
            captured_at_s=1.0,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected.bbox, (100.0, 100.0, 300.0, 350.0))
        self.assertAlmostEqual(selected.confidence, 0.75)

    def test_invalid_boxes_are_ignored(self):
        selected = largest_face(
            "left",
            [(20, 20, 10, 10)],
            [0.9],
            frame_width=640,
            frame_height=480,
            captured_at_s=1.0,
        )

        self.assertIsNone(selected)


class CameraDiscoveryTests(unittest.TestCase):
    def test_only_capture_capable_w19_nodes_are_selected_in_usb_path_order(self):
        devices = [
            Path("/dev/video8"),
            Path("/dev/video3"),
            Path("/dev/video2"),
            Path("/dev/video9"),
        ]
        properties = {
            "/dev/video8": {
                "ID_V4L_PRODUCT": "W19 HD Webcam",
                "ID_V4L_CAPABILITIES": ":capture:",
                "ID_PATH": "pci-usb-0:9:1.0",
            },
            "/dev/video9": {
                "ID_V4L_PRODUCT": "W19 HD Webcam",
                "ID_V4L_CAPABILITIES": ":",
                "ID_PATH": "pci-usb-0:9:1.0",
            },
            "/dev/video2": {
                "ID_MODEL": "W19_HD_Webcam",
                "ID_V4L_CAPABILITIES": ":capture:",
                "ID_PATH": "pci-usb-0:4:1.0",
            },
            "/dev/video3": {
                "ID_V4L_PRODUCT": "Integrated Camera",
                "ID_V4L_CAPABILITIES": ":capture:",
                "ID_PATH": "pci-usb-0:8:1.0",
            },
        }

        selected = discover_matching_cameras(
            "w19",
            device_paths=devices,
            property_reader=lambda path: properties[str(path)],
        )

        self.assertEqual(selected, ["/dev/video2", "/dev/video8"])

    def test_no_product_match_returns_no_camera(self):
        selected = discover_matching_cameras(
            "W19",
            device_paths=[Path("/dev/video0")],
            property_reader=lambda _path: {
                "ID_V4L_PRODUCT": "Integrated Camera",
                "ID_V4L_CAPABILITIES": ":capture:",
            },
        )

        self.assertEqual(selected, [])


class MostBiasedCameraSelectorTests(unittest.TestCase):
    def test_single_available_camera_is_used(self):
        selector = MostBiasedCameraSelector()
        right = observation("right", -0.4)

        self.assertEqual(selector.select([right], now_s=1.0), right)

    def test_larger_horizontal_bias_wins_initial_selection(self):
        selector = MostBiasedCameraSelector()
        left = observation("left", 0.2)
        right = observation("right", -0.6)

        self.assertEqual(selector.select([left, right], now_s=1.0), right)

    def test_camera_switch_requires_margin_hold_and_confirmations(self):
        selector = MostBiasedCameraSelector(
            switch_margin_ratio=0.2,
            min_bias_advantage=0.03,
            switch_confirm_frames=3,
            min_hold_s=0.4,
        )
        left = observation("left", 0.40)
        selector.select([left, observation("right", 0.20)], now_s=1.0)
        challenger = observation("right", -0.70)

        self.assertEqual(selector.select([left, challenger], now_s=1.2).camera_id, "left")
        self.assertEqual(selector.select([left, challenger], now_s=1.5).camera_id, "left")
        self.assertEqual(selector.select([left, challenger], now_s=1.6).camera_id, "left")
        self.assertEqual(selector.select([left, challenger], now_s=1.7).camera_id, "right")

    def test_missing_selected_camera_falls_back_immediately(self):
        selector = MostBiasedCameraSelector(min_hold_s=5.0)
        selector.select([observation("left", 0.8)], now_s=1.0)
        right = observation("right", -0.1)

        self.assertEqual(selector.select([right], now_s=1.1), right)


class FaceYawControllerTests(unittest.TestCase):
    def test_deadband_does_not_move_motor(self):
        neck = SimulatedRosNeckClient()
        neck.enable_yaw()
        controller = FaceYawController(neck, deadband=0.1)

        self.assertIsNone(controller.update(observation("left", 0.05), now_s=1.0))
        self.assertEqual(neck.status().position_deg, 0.0)
        neck.disable_yaw()

    def test_bias_is_sent_as_one_direct_yaw_target(self):
        neck = SimulatedRosNeckClient()
        neck.enable_yaw()
        published = []
        original_publish = neck._publish

        def record_publish(values):
            published.append(list(values))
            original_publish(values)

        neck._publish = record_publish
        controller = FaceYawController(neck, deadband=0.1)

        motion = controller.update(observation("left", 0.8), now_s=1.0)

        self.assertIsNotNone(motion)
        self.assertEqual(motion["goal_deg"], 18.2)
        self.assertEqual(motion["pixel_dx"], 256.0)
        self.assertAlmostEqual(motion["command_delta_rad"], 0.31765, places=5)
        self.assertEqual(len(published), 1)
        self.assertEqual(len(published[0]), 6)
        self.assertAlmostEqual(published[0][2], np.deg2rad(18.2), places=6)
        self.assertEqual(published[0][3:6], [0.0] * 3)
        self.assertFalse(neck.status().moving)
        neck.disable_yaw()

    def test_direct_yaw_target_is_rate_limited_to_once_per_second(self):
        neck = SimulatedRosNeckClient()
        neck.enable_yaw()
        controller = FaceYawController(neck, deadband=0.1, command_interval_s=1.0)

        first = controller.update(observation("left", 0.8), now_s=1.0)
        too_early = controller.update(observation("left", 0.8), now_s=1.5)
        second = controller.update(observation("left", 0.8), now_s=2.0)

        self.assertIsNotNone(first)
        self.assertIsNone(too_early)
        self.assertIsNotNone(second)
        neck.disable_yaw()

class SimpleGuiTests(unittest.TestCase):
    def test_two_camera_frames_are_composed_in_one_window_image(self):
        left_image = np.zeros((480, 640, 3), dtype=np.uint8)
        right_image = np.zeros((480, 640, 3), dtype=np.uint8)
        frames = [
            CameraFrame("left", left_image, 1, 1.0),
            CameraFrame("right", right_image, 1, 1.0),
        ]
        targets = [observation("left", 0.2), observation("right", -0.6)]

        canvas = _compose_display(frames, targets, targets[1], status_text="selected=right")

        self.assertEqual(canvas.shape, (480, 1280, 3))

    def test_cached_camera_panel_remains_when_only_other_camera_has_a_fresh_frame(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cache = {}
        initial = [
            CameraFrame("left", image, 1, 1.0),
            CameraFrame("right", image, 1, 1.0),
        ]
        _update_display_frame_cache(cache, initial, ["left", "right"])

        displayed = _update_display_frame_cache(
            cache,
            [CameraFrame("left", image, 2, 2.0)],
            ["left", "right"],
        )

        self.assertEqual([frame.camera_id for frame in displayed], ["left", "right"])
        self.assertEqual([frame.sequence for frame in displayed], [2, 1])

    def test_missing_camera_gets_a_fixed_size_placeholder_panel(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        displayed = _update_display_frame_cache(
            {},
            [CameraFrame("left", image, 1, 1.0)],
            ["left", "right"],
            placeholder_size=(640, 480),
        )

        self.assertEqual([frame.camera_id for frame in displayed], ["left", "right"])
        self.assertEqual(displayed[1].sequence, 0)
        self.assertEqual(displayed[1].image.shape, (480, 640, 3))
        canvas = _compose_display(displayed, [], None, status_text="waiting")
        self.assertEqual(canvas.shape, (480, 1280, 3))


if __name__ == "__main__":
    unittest.main()

