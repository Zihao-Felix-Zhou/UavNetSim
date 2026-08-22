import unittest

from pydantic import ValidationError

from api.app import StartRequest


class StartRequestTests(unittest.TestCase):
    def test_accepts_valid_uav_altitude_range(self):
        request = StartRequest(uav_min_altitude_m=80.0, uav_max_altitude_m=160.0)

        self.assertEqual(request.uav_min_altitude_m, 80.0)
        self.assertEqual(request.uav_max_altitude_m, 160.0)

    def test_rejects_reversed_uav_altitude_range(self):
        with self.assertRaises(ValidationError):
            StartRequest(uav_min_altitude_m=160.0, uav_max_altitude_m=80.0)


if __name__ == "__main__":
    unittest.main()
