import math

from utils import config


ROUTING_PARAMETER_DEFINITIONS = {
    "Greedy": {
        "hello_interval_s": {
            "label": "HELLO interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "table_entry_lifetime_s": {
            "label": "Neighbor lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "DSDV": {
        "hello_interval_s": {
            "label": "Update interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "purge_interval_s": {
            "label": "Link check interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "table_entry_lifetime_s": {
            "label": "Route lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "GRAD": {
        "table_entry_lifetime_s": {
            "label": "Cost entry lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "OPAR": {
        "cost_weight": {
            "label": "Path cost weight", "unit": "", "default": 0.5,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "lifetime_weight": {
            "label": "Lifetime weight", "unit": "", "default": 0.5,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
    },
    "QRouting": {
        "hello_interval_s": {
            "label": "HELLO interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "learning_rate": {
            "label": "Learning rate", "unit": "", "default": 0.5,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "table_entry_lifetime_s": {
            "label": "Neighbor lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "QFANET": {
        "hello_interval_s": {
            "label": "HELLO interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "learning_rate": {
            "label": "Learning rate", "unit": "", "default": 0.6,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "epsilon": {
            "label": "Exploration epsilon", "unit": "", "default": 0.9,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "sinr_weight": {
            "label": "SINR weight", "unit": "", "default": 0.7,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "history_window": {
            "label": "History window", "unit": "pkt", "default": 10,
            "minimum": 1, "maximum": 1000, "step": 1,
        },
        "table_entry_lifetime_s": {
            "label": "Neighbor lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "QGeo": {
        "hello_interval_s": {
            "label": "HELLO interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "learning_rate": {
            "label": "Learning rate", "unit": "", "default": 0.6,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "maximum_reward": {
            "label": "Destination reward", "unit": "", "default": 10.0,
            "minimum": 0.1, "maximum": 10000.0, "step": 1.0,
        },
        "table_entry_lifetime_s": {
            "label": "Neighbor lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "QMR": {
        "hello_interval_s": {
            "label": "HELLO interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "epsilon": {
            "label": "Initial epsilon", "unit": "", "default": 0.8,
            "minimum": 0.0, "maximum": 1.0, "step": 0.05,
        },
        "epsilon_decay": {
            "label": "Epsilon decay", "unit": "", "default": 0.99,
            "minimum": 0.0, "maximum": 1.0, "step": 0.01,
        },
        "table_entry_lifetime_s": {
            "label": "Neighbor lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
    "Baseline_DRL": {
        "hello_interval_s": {
            "label": "HELLO interval", "unit": "s", "default": 0.5,
            "minimum": 0.05, "maximum": 60.0, "step": 0.05,
        },
        "table_entry_lifetime_s": {
            "label": "Neighbor lifetime", "unit": "s", "default": 2.0,
            "minimum": 0.1, "maximum": 120.0, "step": 0.1,
        },
    },
}


def resolve_routing_parameters(protocol, supplied):
    definitions = ROUTING_PARAMETER_DEFINITIONS.get(protocol, {})
    unknown = set(supplied) - set(definitions)
    if unknown:
        names = ", ".join(sorted(unknown))
        raise ValueError(f"Unsupported {protocol} parameter(s): {names}")

    resolved = {}
    for name, definition in definitions.items():
        value = float(supplied.get(name, definition["default"]))
        if not math.isfinite(value):
            raise ValueError(f"{definition['label']} must be finite")
        if not definition["minimum"] <= value <= definition["maximum"]:
            raise ValueError(
                f"{definition['label']} must be between "
                f"{definition['minimum']} and {definition['maximum']}"
            )
        resolved[name] = value
    return resolved


def routing_parameter(name, default):
    return float(config.ROUTING_PROTOCOL_PARAMETERS.get(name, default))


def routing_interval_us(name, default_seconds):
    return routing_parameter(name, default_seconds) * 1e6
