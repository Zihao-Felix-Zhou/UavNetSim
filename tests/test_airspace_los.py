import unittest

from scene.airspace import Airspace
from scene.models import EnuPoint, GeoAnchor, SceneFeature, SceneModel


def _scene_with_building():
    return SceneModel(
        name="LoS test scene",
        anchor=GeoAnchor(latitude=0.0, longitude=0.0),
        size_x=20.0,
        size_y=20.0,
        features=[
            SceneFeature(
                id="building-1",
                category="building",
                footprint=[
                    EnuPoint(x=8.0, y=8.0),
                    EnuPoint(x=12.0, y=8.0),
                    EnuPoint(x=12.0, y=12.0),
                    EnuPoint(x=8.0, y=12.0),
                ],
                height=5.0,
            )
        ],
    )


class AirspaceLosTests(unittest.TestCase):
    def test_explicit_flight_altitude_limits_position_sampling_and_motion(self):
        airspace = Airspace(
            _scene_with_building(),
            max_height=20.0,
            min_flight_height=8.0,
            max_flight_height=12.0,
        )

        positions = airspace.random_positions(seed=2025, count=4, minimum_separation=0.0)
        self.assertTrue(all(8.0 <= position[2] <= 12.0 for position in positions))
        resolved, velocity, _collision = airspace.resolve_motion(
            [2.0, 2.0, 10.0], [2.0, 2.0, 14.0], [0.0, 0.0, 2.0]
        )
        self.assertEqual(resolved[2], 12.0)
        self.assertEqual(velocity[2], -2.0)

    def test_radio_los_uses_physical_building_without_navigation_clearance(self):
        airspace = Airspace(
            _scene_with_building(),
            max_height=20.0,
            building_clearance=1.0,
            boundary_clearance=1.0,
        )
        start = [2.0, 7.5, 3.0]
        end = [18.0, 7.5, 3.0]

        self.assertIsNotNone(airspace.path_collision(start, end))
        self.assertTrue(airspace.has_line_of_sight(start, end))

    def test_radio_los_detects_building_and_allows_path_above_roof(self):
        airspace = Airspace(_scene_with_building(), max_height=20.0)

        self.assertFalse(airspace.has_line_of_sight([2.0, 10.0, 3.0], [18.0, 10.0, 3.0]))
        self.assertTrue(airspace.has_line_of_sight([2.0, 10.0, 6.0], [18.0, 10.0, 6.0]))


if __name__ == "__main__":
    unittest.main()
