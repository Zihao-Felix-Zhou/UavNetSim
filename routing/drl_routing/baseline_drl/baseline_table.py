from simulator.log import logger
from routing.base.base_table import BaseTable
from utils.util_function import euclidean_distance_3d


class BaselineDrlNeighborTable(BaseTable):
    """Neighbor table used by the built-in baseline DRL routing protocol."""

    def __init__(self, env, my_drone):
        super().__init__(env, my_drone)
        # Compatibility alias for code that still refers to neighbor_table.
        self.neighbor_table = self.table

    def add_item(self, hello_packet, cur_time):
        drone_id = hello_packet.src_drone.identifier
        position = hello_packet.cur_position
        velocity = getattr(hello_packet, "velocity", None)
        energy = getattr(hello_packet, "residual_energy", None)
        self.table[drone_id] = [position, velocity, energy, cur_time]

    def is_neighbor(self, certain_drone):
        drone_id = getattr(certain_drone, "identifier", certain_drone)
        return self.is_item(drone_id)

    def get_neighbor_position(self, certain_drone):
        drone_id = getattr(certain_drone, "identifier", certain_drone)
        if self.is_item(drone_id):
            return self.table[drone_id][0]
        raise RuntimeError("This drone is not my neighbor")

    def print_item(self, my_drone):
        logger.info("|----------Neighbor Table of: %s ----------|", my_drone.identifier)
        for key in self.table:
            logger.info(
                "Neighbor: %s, position is: %s, updated time is: %s",
                key,
                self.table[key][0],
                self.table[key][-1],
            )
        logger.info("|-----------------------------------------------------------------|")

    def print_neighbor(self, my_drone):
        self.print_item(my_drone)

    def best_neighbor(self, my_drone, dst_drone):
        best_distance = euclidean_distance_3d(my_drone.coords, dst_drone.coords)
        best_id = my_drone.identifier

        for key in self.table:
            next_hop_position = self.table[key][0]
            temp_distance = euclidean_distance_3d(next_hop_position, dst_drone.coords)
            if temp_distance < best_distance:
                best_distance = temp_distance
                best_id = key

        return best_id
