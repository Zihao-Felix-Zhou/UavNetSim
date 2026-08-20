import queue
import threading

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import simpy

from utils.radio import routing_neighbor_distance
from routing.drl_routing.baseline_drl.baseline_drl import BaselineDrl
from routing.drl_routing.baseline_drl.baseline_packet import BaselineDrlHelloPacket
from routing.drl_routing.baseline_drl.baseline_table import BaselineDrlNeighborTable
from simulator.simulator import Simulator
from utils import config
from utils.util_function import euclidean_distance_3d

class DroneRoutingEnv(gym.Env):
    """Shared Gymnasium environment for DRL-based routing protocols."""

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        reward_fn=None,
        obs_fn=None,
        obs_dim=None,
        hello_packet_cls=None,
        neighbor_table_cls=None,
        invalid_action_fn=None,
        routing_protocol="BASELINE_DRL",
        routing_protocol_cls=None,
    ):
        super().__init__()
        config.ROUTING_PROTOCOL = routing_protocol
        config.DRL_ROUTING_PROTOCOL_CLASS = (
            routing_protocol_cls
            or getattr(config, "DRL_ROUTING_PROTOCOL_CLASS", None)
            or BaselineDrl
        )
        config.DRL_HELLO_PACKET_CLASS = (
            hello_packet_cls
            or getattr(config, "DRL_HELLO_PACKET_CLASS", None)
            or BaselineDrlHelloPacket
        )
        config.DRL_NEIGHBOR_TABLE_CLASS = (
            neighbor_table_cls
            or getattr(config, "DRL_NEIGHBOR_TABLE_CLASS", None)
            or BaselineDrlNeighborTable
        )

        self.N = config.NUMBER_OF_DRONES
        self.max_comm_range = routing_neighbor_distance()
        self.action_space = spaces.Discrete(self.N)

        self.obs_fn = obs_fn
        if obs_fn is not None and obs_dim is not None:
            self.observation_space = spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(obs_dim,),
                dtype=np.float32,
            )
        else:
            default_obs_dim = 13 + 6 * self.N
            self.observation_space = spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(default_obs_dim,),
                dtype=np.float32,
            )

        self.reward_fn = reward_fn if reward_fn else self._default_reward_fn
        self.invalid_action_fn = invalid_action_fn if invalid_action_fn else self._default_invalid_action_fn
        self.sim_thread = None
        self.obs_queue = None
        self.action_queue = None
        self.last_context = None
        self.simulator = None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._kill_sim_thread()

        self.obs_queue = queue.Queue()
        self.action_queue = queue.Queue()
        self.last_context = None

        sim_seed = seed if seed is not None else np.random.randint(100000)
        self.sim_thread = threading.Thread(target=self._run_simulation, args=(sim_seed,), daemon=True)
        self.sim_thread.start()

        msg_type, context = self.obs_queue.get(block=True)
        if msg_type == "DONE":
            return np.zeros(self.observation_space.shape, dtype=np.float32), {
                "action_mask": np.zeros(self.N, dtype=np.int8)
            }

        self.last_context = context
        return self._extract_obs(context), {
            "sim_time": context["sim_time"],
            "action_mask": self._get_action_mask(context),
        }

    def step(self, action: int):
        action = int(action)

        if self.last_context is not None:
            mask = self._get_action_mask(self.last_context)
            if mask[action] == 0:
                action = self.invalid_action_fn(action, mask, self.last_context)

        reward = self.reward_fn(action, self.last_context)
        self.action_queue.put(action)
        msg_type, context = self.obs_queue.get(block=True)

        terminated, truncated = False, False
        if msg_type == "DONE":
            truncated = True
            self.last_context = None
            return np.zeros(self.observation_space.shape, dtype=np.float32), float(reward), terminated, truncated, {
                "action_mask": np.zeros(self.N, dtype=np.int8)
            }

        self.last_context = context
        return self._extract_obs(context), float(reward), terminated, truncated, {
            "sim_time": context["sim_time"],
            "action_mask": self._get_action_mask(context),
        }

    def close(self):
        self._kill_sim_thread()

    def _run_simulation(self, seed):
        try:
            env = simpy.Environment()
            sim = Simulator(
                seed=seed,
                env=env,
                n_drones=self.N,
                action_queue=self.action_queue,
                obs_queue=self.obs_queue,
            )
            self.simulator = sim
            env.run(until=config.SIM_TIME)
        except simpy.Interrupt:
            pass
        except Exception as exc:
            print(f"[Simulation Error] {exc}")
        finally:
            if self.simulator is not None:
                self.simulator.close()
            self.obs_queue.put(("DONE", None))

    def _kill_sim_thread(self):
        if self.sim_thread and self.sim_thread.is_alive():
            try:
                self.action_queue.put(-1, block=False)
            except Exception:
                pass
            self.sim_thread.join(timeout=2.0)

    def _extract_obs(self, context):
        if self.obs_fn is not None:
            return self.obs_fn(context, self.N)
        return self._default_extract_obs(context)

    def _default_extract_obs(self, context):
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        idx = 0
        curr_id = context["current_drone_id"]
        dest_id = context["dest_id"]
        positions = context["positions"]
        velocities = context["velocities"]
        energies = context["energies"]
        queue_sizes = context["queue_sizes"]
        valid_neighbors = context["valid_neighbors"]

        obs[idx] = curr_id / max(1, self.N - 1)
        idx += 1
        obs[idx:idx + 3] = np.array(positions[curr_id]) / max(config.MAP_LENGTH, config.MAP_WIDTH, config.MAP_HEIGHT)
        idx += 3
        obs[idx:idx + 3] = np.clip(np.array(velocities[curr_id]) / 60.0, -1.0, 1.0)
        idx += 3
        obs[idx] = energies[curr_id] / config.INITIAL_ENERGY
        idx += 1
        obs[idx] = queue_sizes[curr_id] / config.MAX_QUEUE_SIZE
        idx += 1

        obs[idx] = dest_id / max(1, self.N - 1)
        idx += 1
        obs[idx:idx + 3] = np.array(positions[dest_id]) / max(config.MAP_LENGTH, config.MAP_WIDTH, config.MAP_HEIGHT)
        idx += 3

        dist_curr_to_dest = euclidean_distance_3d(positions[curr_id], positions[dest_id])
        for drone_id in range(self.N):
            is_neighbor = 1.0 if drone_id in valid_neighbors else 0.0
            obs[idx] = is_neighbor
            idx += 1
            if is_neighbor:
                obs[idx] = euclidean_distance_3d(positions[curr_id], positions[drone_id]) / self.max_comm_range
                idx += 1
                dist_to_dest = euclidean_distance_3d(positions[drone_id], positions[dest_id])
                obs[idx] = np.clip((dist_curr_to_dest - dist_to_dest) / self.max_comm_range, -1.0, 1.0)
                idx += 1
                obs[idx] = 1.0
                idx += 1
                obs[idx] = queue_sizes[drone_id] / config.MAX_QUEUE_SIZE
                idx += 1
                obs[idx] = energies[drone_id] / config.INITIAL_ENERGY
                idx += 1
            else:
                obs[idx:idx + 5] = [
                    0.0,
                    0.0,
                    -1.0,
                    queue_sizes[drone_id] / config.MAX_QUEUE_SIZE,
                    energies[drone_id] / config.INITIAL_ENERGY,
                ]
                idx += 5
        return obs

    def _get_action_mask(self, context):
        mask = np.zeros(self.N, dtype=np.int8)
        for valid_id in context["valid_neighbors"]:
            mask[valid_id] = 1
        return mask

    def action_masks(self) -> np.ndarray:
        if self.last_context is not None:
            return self._get_action_mask(self.last_context)
        return np.zeros(self.N, dtype=np.int8)

    @staticmethod
    def _default_invalid_action_fn(action, mask, context):
        valid_actions = np.where(mask == 1)[0]
        if len(valid_actions) > 0:
            return int(np.random.choice(valid_actions))
        return action

    def _default_reward_fn(self, action: int, context: dict) -> float:
        return 0.0
