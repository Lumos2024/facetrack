import hashlib
import unittest

from neck_face_tracker.tracker import DEFAULT_MODEL, build_parser


EXPECTED_MODEL_SHA256 = "219f490a97d5722972e1660725fa7cfab54e46c944b833debc28c2302aa1be93"


class PackageTests(unittest.TestCase):
    def test_bundled_model_exists_and_matches_expected_file(self):
        self.assertTrue(DEFAULT_MODEL.is_file())
        digest = hashlib.sha256(DEFAULT_MODEL.read_bytes()).hexdigest()
        self.assertEqual(digest, EXPECTED_MODEL_SHA256)

    def test_default_cli_uses_two_cameras_and_detection_only(self):
        args = build_parser().parse_args([])

        self.assertEqual(args.cameras, ["0", "1"])
        self.assertFalse(args.control)
        self.assertEqual(args.model, str(DEFAULT_MODEL))
        self.assertEqual(args.target_selected_topic, "/target_selected")
        self.assertEqual(args.selected_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
