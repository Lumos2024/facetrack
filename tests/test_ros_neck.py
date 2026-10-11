import math
import time
import unittest

from neck_face_tracker.ros_neck import (
    CMD_DISABLE,
    CMD_ENABLE,
    MOTOR1_HOLD_KD,
    MOTOR1_HOLD_KP,
    MOTOR1_MOVE_KD,
    MOTOR1_MOVE_KP,
    PUBLISH_PERIOD_S,
    RosNeckClient,
    SimulatedRosNeckClient,
    neck_command_payload,
    neck_motion_payload,
)


class RosNeckProtocolTests(unittest.TestCase):
    def test_motor1_command_does_not_target_motor2_or_motor3_positions(self):
        payload = neck_command_payload(CMD_ENABLE)

        self.assertEqual(len(payload), 12)
        self.assertTrue(all(math.isnan(value) for value in payload[:3]))
        self.assertEqual(payload[6:9], [float(CMD_ENABLE), 0.0, 0.0])

    def test_motor1_motion_uses_nan_for_other_motor_positions(self):
        payload = neck_motion_payload(0.1, 0.2, 0.3)

        self.assertEqual(len(payload), 12)
        self.assertEqual(payload[0], 0.1)
        self.assertTrue(math.isnan(payload[1]))
        self.assertTrue(math.isnan(payload[2]))
        self.assertEqual(payload[3:6], [0.2, 0.0, 0.0])
        self.assertEqual(payload[9:12], [0.3, 0.0, 0.0])

    def test_simulated_motor_requires_enable_and_clamps_jog(self):
        client = SimulatedRosNeckClient()
        with self.assertRaisesRegex(RuntimeError, "Enable"):
            client.jog_motor1(3.0, 1.0)

        client.enable_motor1()
        self.assertTrue(client.status().enabled)
        motion = client.jog_motor1(20.0, 0.5)

        self.assertEqual(motion["delta_deg"], 10.0)
        self.assertTrue(client.status().moving)
        client.disable_motor1()
        self.assertFalse(client.status().enabled)

    def test_absolute_move_matches_current_mit_gui_profile(self):
        client = SimulatedRosNeckClient()
        client.enable_motor1()

        motion = client.move_motor1(45.0, 2.0)

        self.assertEqual(motion["goal_deg"], 45.0)
        self.assertEqual(motion["duration_s"], 2.0)
        self.assertEqual(PUBLISH_PERIOD_S, 0.01)
        self.assertEqual((MOTOR1_HOLD_KP, MOTOR1_HOLD_KD), (30.0, 1.0))
        self.assertEqual((MOTOR1_MOVE_KP, MOTOR1_MOVE_KD), (40.0, 1.0))
        client.disable_motor1()

    def test_trajectory_uses_roughly_100_hz_deadline_schedule(self):
        client = SimulatedRosNeckClient()
        client.enable_motor1()
        published_at = []
        original_publish = client._publish

        def record_publish(values):
            if values and math.isfinite(values[0]):
                published_at.append(time.monotonic())
            original_publish(values)

        client._publish = record_publish
        client.move_motor1(3.0, 0.5)
        deadline = time.monotonic() + 1.0
        while client.status().moving and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertGreaterEqual(len(published_at), 45)
        self.assertLessEqual(len(published_at), 60)
        self.assertAlmostEqual(published_at[-1] - published_at[0], 0.5, delta=0.08)
        client.disable_motor1()

    def test_state_code_one_is_enabled_not_fault(self):
        client = SimulatedRosNeckClient()
        client.update_state([0.0, 0.0, 0.0, 25.0, 25.0, 1.0, 1.0, 1.0])

        client.enable_motor1()

        self.assertTrue(client.status().enabled)

    def test_target_selected_bool_is_continuously_stored(self):
        class Message:
            data = True

        client = RosNeckClient(auto_start=False)
        client._target_selected_callback(Message())
        self.assertTrue(client.target_selected())

        Message.data = False
        client._target_selected_callback(Message())
        self.assertFalse(client.target_selected())

    def test_selected_rad_is_published_exactly_once(self):
        client = SimulatedRosNeckClient()
        client.enable_motor1()
        published = []
        original_publish = client._publish

        def record_publish(values):
            published.append(list(values))
            original_publish(values)

        client._publish = record_publish
        client.move_motor1(10.0, 0.5)
        time.sleep(0.03)
        published.clear()
        command = client.send_motor1_position_once(0.25)
        count_after_selected_command = len(published)
        time.sleep(0.05)

        self.assertEqual(count_after_selected_command, 1)
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0][0], 0.25)
        self.assertEqual(command["goal_rad"], 0.25)
        self.assertFalse(client.status().moving)
        client.disable_motor1()

    def test_selected_rad_rejects_non_finite_value(self):
        client = SimulatedRosNeckClient()
        client.enable_motor1()

        with self.assertRaisesRegex(ValueError, "finite"):
            client.send_motor1_position_once(math.nan)

        client.disable_motor1()

    def test_feedback_timeout_marks_connection_stale(self):
        client = RosNeckClient(auto_start=False)
        client._available = True
        client.update_state([0.0, 0.0, 0.0, 25.0, 25.0, 0.0, 0.0, 1.0])
        client._last_feedback_s = time.monotonic() - 2.0

        status = client.status()

        self.assertFalse(status.connected)
        self.assertEqual(status.message, "Motor 1 feedback timeout")

    def test_trajectory_stops_when_feedback_is_lost(self):
        client = SimulatedRosNeckClient()
        client.enable_motor1()
        client.jog_motor1(3.0, 1.0)
        client._last_feedback_s = time.monotonic() - 2.0

        time.sleep(0.05)

        self.assertFalse(client.status().moving)
        self.assertIn(client.status().message, {"Motor 1 feedback timeout", "trajectory stopped: Motor 1 feedback lost or disabled"})


if __name__ == "__main__":
    unittest.main()
