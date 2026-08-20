import os

import matplotlib

matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure

from utils.radio import routing_neighbor_distance
from routing.drl_routing.baseline_drl.baseline_drl import BaselineDrl
from routing.drl_routing.baseline_drl.baseline_packet import BaselineDrlHelloPacket
from routing.drl_routing.baseline_drl.baseline_table import BaselineDrlNeighborTable
from utils import config
from utils.util_function import euclidean_distance_3d

# ---------------------------------------------------------------------
# Unified DRL user entry
# Swap the three class bindings below if you want to use a custom DRL
# routing protocol implemented under routing/drl_routing/<your_protocol>/.
# ---------------------------------------------------------------------
ROUTING_PROTOCOL_NAME = "BASELINE_DRL"
ROUTING_PROTOCOL_CLASS = BaselineDrl
HELLO_PACKET_CLASS = BaselineDrlHelloPacket
NEIGHBOR_TABLE_CLASS = BaselineDrlNeighborTable

# ---------------------------- RL setup ---------------------------- #
MODEL_SAVE_DIR = "./ppo_routing_logs/"
MODEL_SAVE_NAME = "ppo_uav_routing"
RL_ALGORITHM_CLASS = PPO
RL_ALGORITHM_POLICY = "MlpPolicy"
RL_ALGORITHM_KWARGS = {
    "learning_rate": 3e-4,
    "n_steps": 1024,
    "batch_size": 64,
    "verbose": 1,
    "device": "cpu",
}
TOTAL_TIMESTEPS = 50000
N_ENVS = 4
OBS_DIM = 9 + config.NUMBER_OF_DRONES * 4


class UserHelloPacket(HELLO_PACKET_CLASS):
    """Default packet extension point for the active DRL protocol."""


class UserNeighborTable(NEIGHBOR_TABLE_CLASS):
    """Default neighbor-table extension point for the active DRL protocol."""


# ------------------------- User hook area ------------------------- #
def custom_reward_fn(action: int, context: dict) -> float:
    curr_id = context["current_drone_id"]
    dest_id = context["dest_id"]
    valid_neighbors = context["valid_neighbors"]
    positions = context["positions"]
    queue_sizes = context["queue_sizes"]
    energies = context["energies"]

    if action not in valid_neighbors or action == curr_id:
        return -10.0

    if action == dest_id:
        return 50.0

    max_comm = routing_neighbor_distance()
    dist_now = euclidean_distance_3d(positions[curr_id], positions[dest_id])
    dist_next = euclidean_distance_3d(positions[action], positions[dest_id])
    advance = (dist_now - dist_next) / max_comm

    queue_penalty = queue_sizes[action] / config.MAX_QUEUE_SIZE
    energy_penalty = 1.0 - (energies[action] / config.INITIAL_ENERGY)
    reward = (advance * 5.0) - (queue_penalty * 2.0) - energy_penalty - 0.1
    return float(reward)



def random_fallback(action, mask, context):
    valid_actions = np.where(mask == 1)[0]
    if len(valid_actions) > 0:
        return int(np.random.choice(valid_actions))
    return action



def greedy_fallback(action, mask, context):
    valid_actions = np.where(mask == 1)[0]
    if len(valid_actions) == 0:
        return action

    dest_pos = context["positions"][context["dest_id"]]
    best_action = action
    best_dist = float("inf")
    for candidate in valid_actions:
        dist = euclidean_distance_3d(context["positions"][candidate], dest_pos)
        if dist < best_dist:
            best_dist = dist
            best_action = int(candidate)
    return best_action



def reject_fallback(action, mask, context):
    return action



def custom_obs_fn(context: dict, n_drones: int) -> np.ndarray:
    obs = np.zeros(9 + n_drones * 4, dtype=np.float32)
    idx = 0
    curr_id = context["current_drone_id"]
    dest_id = context["dest_id"]
    positions = context["positions"]
    energies = context["energies"]
    queue_sizes = context["queue_sizes"]
    valid_neighbors = context["valid_neighbors"]

    max_map = max(config.MAP_LENGTH, config.MAP_WIDTH, config.MAP_HEIGHT)
    max_comm = routing_neighbor_distance()

    obs[idx:idx + 3] = np.array(positions[curr_id]) / max_map
    idx += 3
    obs[idx] = energies[curr_id] / config.INITIAL_ENERGY
    idx += 1
    obs[idx] = queue_sizes[curr_id] / config.MAX_QUEUE_SIZE
    idx += 1

    obs[idx:idx + 3] = np.array(positions[dest_id]) / max_map
    idx += 3
    obs[idx] = dest_id / max(1, n_drones - 1)
    idx += 1

    dist_to_dest = euclidean_distance_3d(positions[curr_id], positions[dest_id])
    for drone_id in range(n_drones):
        is_neighbor = 1.0 if drone_id in valid_neighbors else 0.0
        obs[idx] = is_neighbor
        idx += 1
        if is_neighbor:
            dist_candidate = euclidean_distance_3d(positions[drone_id], positions[dest_id])
            obs[idx] = np.clip((dist_to_dest - dist_candidate) / max_comm, -1.0, 1.0)
        else:
            obs[idx] = 0.0
        idx += 1
        obs[idx] = queue_sizes[drone_id] / config.MAX_QUEUE_SIZE
        idx += 1
        obs[idx] = energies[drone_id] / config.INITIAL_ENERGY
        idx += 1

    return obs



def build_env_kwargs(invalid_action_fn=random_fallback):
    return {
        "reward_fn": custom_reward_fn,
        "obs_fn": custom_obs_fn,
        "obs_dim": OBS_DIM,
        "hello_packet_cls": UserHelloPacket,
        "neighbor_table_cls": UserNeighborTable,
        "invalid_action_fn": invalid_action_fn,
        "routing_protocol": ROUTING_PROTOCOL_NAME,
        "routing_protocol_cls": ROUTING_PROTOCOL_CLASS,
    }


def apply_runtime_config():
    config.ROUTING_PROTOCOL = ROUTING_PROTOCOL_NAME
    config.DRL_ROUTING_PROTOCOL_CLASS = ROUTING_PROTOCOL_CLASS
    config.DRL_HELLO_PACKET_CLASS = UserHelloPacket
    config.DRL_NEIGHBOR_TABLE_CLASS = UserNeighborTable



def custom_train_fn(env_kwargs):
    from routing.drl_routing.drone_routing_env import DroneRoutingEnv

    log_dir = MODEL_SAVE_DIR
    os.makedirs(log_dir, exist_ok=True)

    vec_env = make_vec_env(DroneRoutingEnv, n_envs=N_ENVS, env_kwargs=env_kwargs)
    model = RL_ALGORITHM_CLASS(RL_ALGORITHM_POLICY, vec_env, **RL_ALGORITHM_KWARGS)

    new_logger = configure(log_dir, ["stdout", "csv"])
    model.set_logger(new_logger)

    print(
        f"\nStarting training (Algorithm: {RL_ALGORITHM_CLASS.__name__}, "
        f"Total timesteps: {TOTAL_TIMESTEPS})."
    )
    model.learn(total_timesteps=TOTAL_TIMESTEPS)

    save_path = os.path.join(log_dir, MODEL_SAVE_NAME)
    model.save(save_path)
    print(f"Model saved to: {save_path}")

    vec_env.close()
    _plot_training_curves(log_dir)



def custom_load_fn(model_path):
    model = RL_ALGORITHM_CLASS.load(model_path)
    print(f"Model loaded successfully: {model_path} (Algorithm: {RL_ALGORITHM_CLASS.__name__})")
    return model



def _plot_training_curves(log_dir):
    csv_path = os.path.join(log_dir, "progress.csv")
    if not os.path.exists(csv_path):
        print("Training log CSV file not found, skipping plot.")
        return

    try:
        df = pd.read_csv(csv_path)
        plt.figure(figsize=(15, 4))

        plt.subplot(1, 3, 1)
        if "rollout/ep_rew_mean" in df.columns:
            df_rew = df.dropna(subset=["rollout/ep_rew_mean"])
            plt.plot(df_rew["time/total_timesteps"], df_rew["rollout/ep_rew_mean"], c="blue", marker=".")
            plt.title("Average Episode Reward")
            plt.xlabel("Timesteps")
            plt.ylabel("Reward")
            plt.grid()

        plt.subplot(1, 3, 2)
        if "train/value_loss" in df.columns:
            df_vloss = df.dropna(subset=["train/value_loss"])
            plt.plot(df_vloss["time/total_timesteps"], df_vloss["train/value_loss"], c="red")
            plt.title("Value Loss (Critic)")
            plt.xlabel("Timesteps")
            plt.ylabel("Loss")
            plt.grid()

        plt.subplot(1, 3, 3)
        if "train/policy_gradient_loss" in df.columns:
            df_ploss = df.dropna(subset=["train/policy_gradient_loss"])
            plt.plot(df_ploss["time/total_timesteps"], df_ploss["train/policy_gradient_loss"], c="green")
            plt.title("Policy Gradient Loss (Actor)")
            plt.xlabel("Timesteps")
            plt.ylabel("Loss")
            plt.grid()

        plt.tight_layout()
        curve_path = os.path.join(log_dir, "training_curves.png")
        plt.savefig(curve_path)
        print(f"Training curves saved to: {curve_path}")
    except Exception as exc:
        print(f"Failed to plot training curves: {exc}")
