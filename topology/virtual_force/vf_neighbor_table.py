import math
import numpy as np
from collections import defaultdict
from utils import config
from utils.util_function import euclidean_distance_3d


class VfNeighborTable:
    """
    Neighbor table of motion controller

    Neighbors in this algorithm is mainly used to calculate repulsive force

    Attributes:
        env: simpy environment
        my_drone: the drone that installed the GPSR
        neighbor_table: a dictionary, used to store the neighbor's information
        entry_life_time: lifetime of each item in the neighbor table
        k: The elastic coefficient of a spring
        desired_distance: when the distance between two nodes is below 'desired_distance', a repulsive force
                          will be generated

    References:
        [1] Liu. H, et al.,"Simple Movement Control Algorithm for Bi-connectivity in Robotic Sensor Networks,"
            IEEE Journal on Selected Areas in Communications, vol. 28, no. 7, pp. 994-1005, 2010.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/5/20
    Updated at: 2025/3/28
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.neighbor_table = defaultdict(list)
        self.entry_life_time = 5 * 1e6  # unit: us (5s)
        self.k = 1 * 1e7
        self.desired_distance = 80

    def add_neighbor(self, packet, cur_time):
        """
        Update the neighbor table according to the hello packet
        :param packet: the received hello packet or ack packet
        :param cur_time: the moment when the packet is received
        :return: none
        """

        drone_id = packet.src_drone.identifier
        position = packet.cur_position
        self.neighbor_table[drone_id] = [position, cur_time]

    def attractive_force(self):
        """
        Calculate the attractive force applied by center point

        Note from ref [1]: the attractive force is introduced between every drone and the central point of the region,
        so that it leads the drones to move towards the centre of the region. With the attractive force, the drones
        can maintain connectivity among themselves.

        :return: attractive force in three dimensions
        """

        center = [config.MAP_LENGTH / 2, config.MAP_WIDTH / 2, config.MAP_HEIGHT / 2]  # center of the map

        attractive_force_magnitude = [euclidean_distance_3d(self.my_drone.coords, center)] * 3

        # Note: The direction of attractive force is from itself to the center
        attractive_force_direction = [center[0] - self.my_drone.coords[0],
                                      center[1] - self.my_drone.coords[1],
                                      center[2] - self.my_drone.coords[2]]
        norm = [math.sqrt(sum([x ** 2 for x in attractive_force_direction]))] * 3
        attractive_force_direction = [a / b for a, b in zip(attractive_force_direction, norm)]

        attractive_force = [c * d for c, d in zip(attractive_force_magnitude, attractive_force_direction)]

        return attractive_force

    def repulsive_force(self):
        """
        Calculate the repulsive force applied by neighbors

        Note from ref [1]: the repulsive force is introduced between every pair of drones if their distance is less
        than a certain value. With repulsive force, the drones can increase the coverage.

        :return: repulsive force in three dimensions
        """

        repulsive_force = [0, 0, 0]

        for key in self.neighbor_table.keys():
            if key != self.my_drone.identifier:
                neighbor_pos = self.neighbor_table[key][0]
                distance = euclidean_distance_3d(self.my_drone.coords, neighbor_pos)
                if distance <= self.desired_distance:
                    repulsive_force_magnitude = [self.k / distance ** 2 - self.k / self.desired_distance ** 2] * 3

                    # Note: The direction of repulsive force is from neighbor to itself
                    repulsive_force_direction = [self.my_drone.coords[0] - neighbor_pos[0],
                                                 self.my_drone.coords[1] - neighbor_pos[1],
                                                 self.my_drone.coords[2] - neighbor_pos[2]]
                    norm = [math.sqrt(sum([x ** 2 for x in repulsive_force_direction]))] * 3
                    repulsive_force_direction = [a / b for a, b in zip(repulsive_force_direction, norm)]

                    repulsive_force_temp = [c * d for c, d in zip(repulsive_force_magnitude, repulsive_force_direction)]

                    repulsive_force = list(np.array(repulsive_force) + np.array(repulsive_force_temp))

        return repulsive_force

    # get the updated time of certain item
    def get_updated_time(self, drone_id):
        if drone_id not in self.neighbor_table.keys():
            raise RuntimeError('This item is not in the neighbor table')
        else:
            return self.neighbor_table[drone_id][1]

    # delete the specified item
    def remove_neighbor(self, drone_id):
        del self.neighbor_table[drone_id]

    # remove the expired item
    def purge(self):
        if not bool(self.neighbor_table):
            # it means that the neighbor table is empty
            return

        for key in list(self.neighbor_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:  # expired
                self.remove_neighbor(key)
