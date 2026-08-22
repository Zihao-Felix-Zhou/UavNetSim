import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from phy.sionna_rt import HybridSionnaRtChannelModel, free_space_path_gain
from utils import config


class RecordingEventBus:
    def __init__(self):
        self.events = []

    def publish(self, event_type, sim_time_us, **data):
        self.events.append((event_type, sim_time_us, data))


class RecordingClient:
    def __init__(self, gain=0.25):
        self.gain = gain
        self.calls = []
        self.closed = False

    def solve(self, transmitter_positions, receiver_positions):
        self.calls.append((transmitter_positions, receiver_positions))
        transmitter_ids = list(transmitter_positions)
        receiver_ids = list(receiver_positions)
        return {
            "gains": [
                [self.gain for _receiver_id in receiver_ids]
                for _transmitter_id in transmitter_ids
            ],
            "transmitter_ids": transmitter_ids,
            "receiver_ids": receiver_ids,
            "solve_time_ms": 12.5,
        }

    def close(self):
        self.closed = True


class PairVisibility:
    def __init__(self, blocked_pairs):
        self.blocked_pairs = blocked_pairs
        self.calls = []

    def has_line_of_sight(self, start, end):
        pair = (tuple(start), tuple(end))
        self.calls.append(pair)
        return pair not in self.blocked_pairs


def _drones():
    return [
        SimpleNamespace(identifier=0, coords=[0.0, 0.0, 10.0]),
        SimpleNamespace(identifier=1, coords=[10.0, 0.0, 10.0]),
        SimpleNamespace(identifier=2, coords=[20.0, 0.0, 10.0]),
    ]


class HybridChannelTests(unittest.TestCase):
    def test_free_space_path_gain_matches_friis_formula(self):
        distance = 100.0
        frequency = 2.4e9
        wavelength = 299_792_458.0 / frequency

        self.assertTrue(math.isclose(
            free_space_path_gain(distance, frequency),
            (wavelength / (4 * math.pi * distance)) ** 2,
            rel_tol=1e-12,
        ))

    @patch.object(config, "CARRIER_FREQUENCY", 2.4e9)
    def test_hybrid_uses_friis_for_los_and_batches_only_nlos_transmitters(self):
        drones = _drones()
        blocked = {(tuple(drones[1].coords), tuple(drones[2].coords))}
        visibility = PairVisibility(blocked)
        client = RecordingClient()
        event_bus = RecordingEventBus()
        model = HybridSionnaRtChannelModel(event_bus, visibility, client)

        gains = model.gains(0.0, drones, [0, 1], [2])

        self.assertTrue(math.isclose(
            gains[(0, 2)], free_space_path_gain(20.0, 2.4e9), rel_tol=1e-12
        ))
        self.assertEqual(gains[(1, 2)], 0.25)
        self.assertEqual(len(client.calls), 1)
        self.assertEqual(list(client.calls[0][0]), [1])
        self.assertEqual(list(client.calls[0][1]), [2])
        event_type, _sim_time, data = event_bus.events[-1]
        self.assertEqual(event_type, "channel_snapshot")
        self.assertEqual(data["mode"], "hybrid")
        self.assertEqual(data["los_link_count"], 1)
        self.assertEqual(data["nlos_link_count"], 1)
        self.assertEqual(data["rt_matrix_link_count"], 1)

    @patch.object(config, "CHANNEL_SNAPSHOT_INTERVAL", 100.0)
    @patch.object(config, "CHANNEL_SNAPSHOT_DISPLACEMENT", 1.0)
    def test_hybrid_reuses_cached_links(self):
        drones = _drones()
        visibility = PairVisibility(set())
        client = RecordingClient()
        model = HybridSionnaRtChannelModel(RecordingEventBus(), visibility, client)

        first = model.gains(0.0, drones, [0, 1], [2])
        second = model.gains(50.0, drones, [0, 1], [2])

        self.assertEqual(second, first)
        self.assertEqual(len(visibility.calls), 2)
        self.assertEqual(client.calls, [])

    @patch.object(config, "CARRIER_FREQUENCY", 2.4e9)
    def test_hybrid_does_not_cache_incidental_los_rt_results(self):
        drones = _drones()
        blocked = {
            (tuple(drones[0].coords), tuple(drones[1].coords)),
            (tuple(drones[1].coords), tuple(drones[2].coords)),
        }
        visibility = PairVisibility(blocked)
        client = RecordingClient(gain=0.5)
        model = HybridSionnaRtChannelModel(RecordingEventBus(), visibility, client)

        gains = model.gains(0.0, drones, [0, 1], [1, 2])

        self.assertEqual(gains[(0, 1)], 0.5)
        self.assertEqual(gains[(1, 2)], 0.5)
        self.assertTrue(math.isclose(
            gains[(0, 2)], free_space_path_gain(20.0, 2.4e9), rel_tol=1e-12
        ))
        self.assertNotEqual(model._links[(0, 2)].gain, 0.5)


if __name__ == "__main__":
    unittest.main()
