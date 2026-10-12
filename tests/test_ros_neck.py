import math
import time
import unittest

from neck_face_tracker.ros_neck import (
    CMD_DISABLE,
    CMD_ENABLE,
    COMMAND_TOPIC,
    STATE_TOPIC,
    YAW_HOLD_KD,
    YAW_HOLD_KP,
    YAW_MOVE_KD,
    YAW_MOVE_KP,
    PUBLISH_PERIOD_S,
    RosNeckClient,
    SimulatedRosNeckClient,
    neck_command_payload,
    neck_motion_payload,
)


class RosNeckProtocolTests(unittest.TestCase):
    def test_can3_command_uses_six_field_esp32_protocol(self):
        payload = neck_command_payload(CMD_ENABLE, current_yaw_position=0.12)

        self.assertEqual(payload, [0.0, 0.0, 0.12, 0.0, 0.0, float(CMD_ENABLE)])

    def test_can3_motion_uses_six_field_esp32_protocol(self):
        payload = neck_motion_payload(0.1)

        self.assertEqual(payload, [0.0, 0.0, 0.1, 0.0, 0.0, 0.0])

    def test_simulated_motor_requires_enable_and_clamps_jog(self):
        client = SimulatedRosNeckClient()
        with self.assertRaisesRegex(RuntimeError, "Enable"):
            client.jog_yaw(3.0, 1.0)

        client.enable_yaw()
        self.assertTrue(client.status().enabled)
        motion = client.jog_yaw(20.0, 0.5)

        self.assertEqual(motion["delta_deg"], 10.0)
        self.assertTrue(client.status().moving)
        client.disable_yaw()
        self.assertFalse(client.status().enabled)

    def test_absolute_move_matches_current_mit_gui_profile(self):
        client = SimulatedRosNeckClient()
        client.enable_yaw()

        motion = client.move_yaw(45.0, 2.0)

        self.assertEqual(motion["goal_deg"], 45.0)
        self.assertEqual(motion["duration_s"], 2.0)
        self.assertEqual(PUBLISH_PERIOD_S, 0.01)
        self.assertEqual((YAW_HOLD_KP, YAW_HOLD_KD), (80.0, 2.0))
        self.assertEqual((YAW_MOVE_KP, YAW_MOVE_KD), (100.0, 2.0))
        client.disable_yaw()

    def test_trajectory_uses_roughly_100_hz_deadline_schedule(self):
        client = SimulatedRosNeckClient()
        client.enable_yaw()
        published_at = []
        original_publish = client._publish

        def record_publish(values):
            if len(values) >= 3 and math.isfinite(values[2]):
                published_at.append(time.monotonic())
            original_publish(values)

        client._publish = record_publish
        client.move_yaw(3.0, 0.5)
        deadline = time.monotonic() + 1.0
        while client.status().moving and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertGreaterEqual(len(published_at), 45)
        self.assertLessEqual(len(published_at), 60)
        self.assertAlmostEqual(published_at[-1] - published_at[0], 0.5, delta=0.08)
        client.disable_yaw()

    def test_state_code_one_is_enabled_not_fault(self):
        client = SimulatedRosNeckClient()
        client.update_state([0.0, 0.0, 0.0, 25.0, 25.0, 1.0, 1.0, 1.0])

        client.enable_yaw()

        self.assertTrue(client.status().enabled)

    def test_target_selected_bool_is_continuously_stored(self):
        class Message:
            data = True

        client = RosNeckClient(auto_start=False)
        self.assertIsNone(client.target_selected())
        self.assertFalse(client.target_selected_received())
        client._target_selected_callback(Message())
        self.assertTrue(client.target_selected())
        self.assertTrue(client.target_selected_received())

        Message.data = False
        client._target_selected_callback(Message())
        self.assertFalse(client.target_selected())

    def test_can3_state_is_last_group_of_24_values(self):
        client = RosNeckClient(auto_start=False)
        client._available = True
        values = [0.0] * 16 + [0.25, 0.5, 0.75, 29.0, 27.0, 1.0, 1.0, 1.0]

        client.update_state(values)

        status = client.status()
        self.assertTrue(status.connected)
        self.assertEqual(status.motor_id, 3)
        self.assertEqual(status.position_rad, 0.25)
        self.assertEqual(status.velocity_rad_s, 0.5)
        self.assertEqual(client.status_dict()["topic_command"], COMMAND_TOPIC)
        self.assertEqual(client.status_dict()["topic_state"], STATE_TOPIC)

    def test_selected_rad_is_published_exactly_once(self):
        client = SimulatedRosNeckClient()
        client.enable_yaw()
        published = []
        original_publish = client._publish

        def record_publish(values):
            published.append(list(values))
            original_publish(values)

        client._publish = record_publish
        client.move_yaw(10.0, 0.5)
        time.sleep(0.03)
        published.clear()
        command = client.send_yaw_position_once(0.25)
        count_after_selected_command = len(published)
        time.sleep(0.05)

        self.assertEqual(count_after_selected_command, 1)
        self.assertEqual(len(published), 1)
        self.assertEqual(len(published[0]), 6)
        self.assertEqual(published[0][:2], [0.0, 0.0])
        self.assertAlmostEqual(published[0][2], 0.25)
        self.assertEqual(published[0][3:6], [0.0, 0.0, 0.0])
        self.assertEqual(command["goal_rad"], 0.25)
        self.assertFalse(client.status().moving)
        client.disable_yaw()

    def test_selected_rad_allows_explicit_45_degree_can3_target(self):
        client = SimulatedRosNeckClient()
        client.enable_yaw()
        published = []
        original_publish = client._publish

        def record_publish(values):
            published.append(list(values))
            original_publish(values)

        client._publish = record_publish
        command = client.send_yaw_position_once(math.radians(45.0))

        self.assertEqual(len(published), 1)
        self.assertEqual(len(published[0]), 6)
        self.assertEqual(published[0][:2], [0.0, 0.0])
        self.assertAlmostEqual(published[0][2], math.radians(45.0), places=6)
        self.assertEqual(published[0][3:6], [0.0] * 3)
        self.assertEqual(command["goal_deg"], 45.0)
        client.disable_yaw()

    def test_selected_rad_rejects_non_finite_value(self):
        client = SimulatedRosNeckClient()
        client.enable_yaw()

        with self.assertRaisesRegex(ValueError, "finite"):
            client.send_yaw_position_once(math.nan)

        client.disable_yaw()

    def test_feedback_timeout_marks_connection_stale(self):
        client = RosNeckClient(auto_start=False)
        client._available = True
        client.update_state([0.0, 0.0, 0.0, 25.0, 25.0, 0.0, 0.0, 1.0])
        client._last_feedback_s = time.monotonic() - 2.0

        status = client.status()

        self.assertFalse(status.connected)
        self.assertEqual(status.message, "CAN3 yaw feedback timeout")

    def test_trajectory_stops_when_feedback_is_lost(self):
        client = SimulatedRosNeckClient()
        client.enable_yaw()
        client.jog_yaw(3.0, 1.0)
        client._last_feedback_s = time.monotonic() - 2.0

        time.sleep(0.05)

        self.assertFalse(client.status().moving)
        self.assertIn(
            client.status().message,
            {"CAN3 yaw feedback timeout", "trajectory stopped: CAN3 yaw feedback lost or disabled"},
        )


if __name__ == "__main__":
    unittest.main()
