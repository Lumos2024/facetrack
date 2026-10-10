import json
import unittest

from neck_face_tracker.neck_owner import NeckOwnerClaim


class NeckOwnerClaimTests(unittest.TestCase):
    def test_claim_round_trip(self):
        original = NeckOwnerClaim(
            owner_id="yolov8_face_tracker",
            active=True,
            priority=50,
            lease_ms=500,
            sent_at_s=123.5,
        )

        decoded = NeckOwnerClaim.from_json(original.to_json())

        self.assertEqual(decoded, original)

    def test_invalid_claim_is_rejected(self):
        self.assertIsNone(NeckOwnerClaim.from_json("not-json"))
        self.assertIsNone(NeckOwnerClaim.from_json(json.dumps({"active": True})))

    def test_lease_values_are_safely_clamped(self):
        decoded = NeckOwnerClaim.from_json(
            json.dumps(
                {
                    "owner_id": "camera",
                    "active": True,
                    "priority": 9999,
                    "lease_ms": 1,
                }
            )
        )

        self.assertEqual(decoded.priority, 1000)
        self.assertEqual(decoded.lease_ms, 100)


if __name__ == "__main__":
    unittest.main()

