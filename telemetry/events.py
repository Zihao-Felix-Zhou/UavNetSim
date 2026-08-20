from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(slots=True)
class SimulationEvent:
    event_type: str
    sim_time_us: float
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self):
        return asdict(self)

