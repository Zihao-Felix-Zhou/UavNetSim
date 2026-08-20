from collections import deque
from threading import Lock

from telemetry.events import SimulationEvent


class EventBus:
    def __init__(self, history_size=10000):
        self._events = deque(maxlen=history_size)
        self._sequence = 0
        self._lock = Lock()

    def publish(self, event_type, sim_time_us, **data):
        event = SimulationEvent(event_type, float(sim_time_us), data).to_dict()
        with self._lock:
            self._sequence += 1
            event["sequence"] = self._sequence
            self._events.append(event)
        return event

    def since(self, sequence):
        with self._lock:
            return [event.copy() for event in self._events if event["sequence"] > sequence]

    def latest(self, event_type=None):
        with self._lock:
            for event in reversed(self._events):
                if event_type is None or event["event_type"] == event_type:
                    return event.copy()
        return None

    @property
    def sequence(self):
        with self._lock:
            return self._sequence
