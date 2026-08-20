import numpy as np
from collections import defaultdict
from utils import util_function
from routing.base.base_table import BaseTable
from utils.util_function import euclidean_distance_3d
from routing.parameters import routing_parameter
from utils.radio import routing_neighbor_distance


class QFanetTable(BaseTable):
    """
    Q-FANET tables
    """
    """
        The structure of the neighbor table in Q-FANET
        |    Drone id   |  0  |    1     |   2    |  3   |        4           |   5   |     6     |
        | neighbor_id 1 | pos | velocity | energy | sinr | [dst_id:{rewards}] | delay | time step |
        | neighbor_id 2 | pos | velocity | energy | sinr | [dst_id:{rewards}] | delay | time step |
        |       ...     | ... | ...      |
        | neighbor_id N | pos | velocity | energy | sinr | [dst_id:{rewards}] | delay | time step |
    
    Author: abing, xbb992@vip.qq.com
    Created at: 2025/8/20
    Updated at: 2026/3/10
    """
    def __init__(self, env, my_drone, rng_routing):
        super().__init__(env, my_drone)
        self.env = env
        self.my_drone = my_drone
        self.rng_routing = rng_routing
        self.table = defaultdict(lambda: [None, None, 0, [], defaultdict(list), None, 0])
        self.lookback = int(routing_parameter("history_window", 10))
        self.learning_rate = routing_parameter("learning_rate", 0.6)
        self.sinr_weight = routing_parameter("sinr_weight", 0.7)

        n_drones = my_drone.simulator.n_drones
        self.q_table = np.full((n_drones, n_drones), 0.5)
        self.max_comm_range = routing_neighbor_distance()

    def is_item(self, drone_id):
        """Check if the drone is a neighbor"""
        if drone_id not in self.table:
            return False
        # check if the neighbor entry is expired
        if self.table[drone_id][-1] + self.entry_life_time <= self.env.now:
            return False
        dist = euclidean_distance_3d(
            self.my_drone.coords,
            self.table[drone_id][0]
        )
        return dist <= self.max_comm_range

    def add_item(self, hello_packet, cur_time, cur_sinr=None):
        """Add neighbor to the table"""
        src_id = hello_packet.src_drone.identifier
        if src_id == self.my_drone.identifier:
            return
        delay = cur_time - hello_packet.creation_time

        self.table[src_id][0] = hello_packet.cur_position
        self.table[src_id][1] = hello_packet.cur_velocity

        self.table[src_id][2] = hello_packet.src_energy   # 3
        self.table[src_id][3] = cur_sinr  # 4
        self.table[src_id][5] = delay  # 6
        self.table[src_id][6] = cur_time
        # [pos,velocity,energy,sinr,list of reward history,delay,timestamp]

    @staticmethod
    def calculate_eta(sinr):
        """Transform SINR to eta value"""
        if sinr < 8:
            return 0.0
        elif 8 <= sinr < 10:
            return 0.25
        elif 10 <= sinr < 15:
            return 0.5
        elif 15 <= sinr < 17:
            return 0.75
        else:
            return 1.0

    def calculate_velocity_constraint(self, neighbor_id, dst_drone):
        """Calculate Velocity"""
        if not self.is_item(neighbor_id):
            return -1

        d_myself_dst = euclidean_distance_3d(self.my_drone.coords, dst_drone.coords)
        d_neighbor_dst = euclidean_distance_3d(self.table[neighbor_id][0], dst_drone.coords)

        delay = self.table[neighbor_id][5] / 1e6
        return (d_myself_dst - d_neighbor_dst) / delay

    def void_area_judgment(self, dst_drone):
        """Routing hole judgment"""
        self.purge()
        if not self.table:
            return 1

        d_myself_dst = euclidean_distance_3d(self.my_drone.coords, dst_drone.coords)
        for neighbor_id in self.table:
            if not self.is_item(neighbor_id):
                continue
            d_neighbor_dst = euclidean_distance_3d(self.table[neighbor_id][0], dst_drone.coords)
            if d_neighbor_dst <= d_myself_dst:
                return 0
        return 1

    def get_candidate_neighbors(self, dst_drone, packet):
        """Obtain the candidates neighbors"""
        self.purge()
        packet_deadline = (packet.creation_time + packet.deadline - self.env.now) / 1e6
        # calculate the required velocity
        dist_to_dest = util_function.euclidean_distance_3d(self.my_drone.coords, dst_drone.coords)
        required_velocity = dist_to_dest / packet_deadline
        candidates = []
        sub_candidates = []
        for neighbor_id in self.table:
            velocity = self.calculate_velocity_constraint(neighbor_id, dst_drone)
            if velocity > required_velocity:
                candidates.append(neighbor_id)
            elif velocity > 0:
                # velocity smaller than required_velocity but greater than 0
                sub_candidates.append((neighbor_id, velocity))

        return candidates, sub_candidates

    def best_neighbor(self, packet, dst_drone, epsilon=0.1):
        self.purge()
        dst_id = dst_drone.identifier
        if len(self.table.keys()) == 0:
            return self.my_drone.identifier

        candidates, sub_candidates = self.get_candidate_neighbors(dst_drone, packet)
        if len(candidates) > 0:
            # Q-Noise+
            if self.rng_routing.random() > epsilon:
                best_id = self.rng_routing.choice(list(candidates))
            else:
                best_id = max(candidates, key=lambda x: self.q_table[x][dst_id])
        elif len(sub_candidates) > 0:
            best_id = max(sub_candidates, key=lambda x: x[1])[0]
        else:
            # routing hole
            best_id = self.my_drone.identifier
        return best_id

    def update_q_value(self, next_hop_id, dst_id, reward, sinr_eta):
        """Q-Noise+ update"""
        if not self.is_item(next_hop_id):
            return
        # get the lookback rewards history
        lookback_rewards = self.table[next_hop_id][4].get(dst_id, [])
        lookback_rewards = lookback_rewards[-self.lookback:]
        # generate random weights
        weights = [self.rng_routing.random() for _ in range(self.lookback)]
        weights = [w / sum(weights) for w in weights]

        while len(lookback_rewards) < self.lookback:
            lookback_rewards.insert(0, 0)
        weighted_historical = sum(w * r for w, r in zip(weights, lookback_rewards))

        self.q_table[next_hop_id][dst_id] = (1 - self.learning_rate) * weighted_historical + \
                                            self.learning_rate * reward + \
                                            self.sinr_weight * sinr_eta

        self.table[next_hop_id][4][dst_id].append(reward)
        self.table[next_hop_id][4][dst_id] = self.table[next_hop_id][4][dst_id][-self.lookback:]


