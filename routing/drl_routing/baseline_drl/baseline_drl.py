import copy
import random

import simpy

from entities.packet import AckPacket, DataPacket
from routing.drl_routing.baseline_drl.baseline_packet import BaselineDrlHelloPacket
from routing.drl_routing.baseline_drl.baseline_table import BaselineDrlNeighborTable
from routing.parameters import routing_interval_us
from simulator.log import logger
from topology.virtual_force.vf_packet import VfPacket
from utils import config


class BaselineDrl:
    """Built-in DRL-based routing protocol with configurable state/reward hooks."""

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.rng_routing = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 10)
        self.hello_interval = routing_interval_us("hello_interval_s", 0.5)
        self.check_interval = 0.6 * 1e6
        self.hello_packet_cls = getattr(config, "DRL_HELLO_PACKET_CLASS", None) or BaselineDrlHelloPacket
        table_cls = getattr(config, "DRL_NEIGHBOR_TABLE_CLASS", None) or BaselineDrlNeighborTable
        self.neighbor_table = table_cls(self.simulator.env, my_drone)
        self.action_queue = getattr(self.simulator, "action_queue", None)
        self.obs_queue = getattr(self.simulator, "obs_queue", None)

        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())

    def broadcast_hello_packet(self, my_drone):
        config.GL_ID_HELLO_PACKET += 1
        channel_id = self.my_drone.channel_assigner.channel_assign()
        hello_packet = self.hello_packet_cls(
            src_drone=my_drone,
            creation_time=self.simulator.env.now,
            id_hello_packet=config.GL_ID_HELLO_PACKET,
            hello_packet_length=config.HELLO_PACKET_LENGTH,
            simulator=self.simulator,
            channel_id=channel_id,
        )
        hello_packet.transmission_mode = 1

        logger.info(
            "At time: %s (us) ---- UAV: %s has a baseline DRL hello packet to broadcast",
            self.simulator.env.now,
            self.my_drone.identifier,
        )

        self.simulator.metrics.control_packet_num += 1
        self.my_drone.transmitting_queue.put(hello_packet)

    def broadcast_hello_packet_periodically(self):
        while True:
            self.broadcast_hello_packet(self.my_drone)
            jitter = self.rng_routing.randint(1000, 2000)
            yield self.simulator.env.timeout(self.hello_interval + jitter)

    def _extract_context(self, packet):
        my_id = self.my_drone.identifier
        dest_id = packet.dst_drone.identifier
        positions = {}
        velocities = {}
        energies = {}
        queue_sizes = {}
        valid_neighbors = []

        for drone in self.simulator.drones:
            drone_id = drone.identifier
            positions[drone_id] = drone.coords
            velocities[drone_id] = drone.velocity
            energies[drone_id] = drone.residual_energy
            queue_sizes[drone_id] = drone.transmitting_queue.qsize()
            if drone_id != my_id and self.neighbor_table.is_neighbor(drone):
                valid_neighbors.append(drone_id)

        return {
            "sim_time": self.simulator.env.now,
            "current_drone_id": my_id,
            "dest_id": dest_id,
            "valid_neighbors": valid_neighbors,
            "positions": positions,
            "velocities": velocities,
            "energies": energies,
            "queue_sizes": queue_sizes,
        }

    def next_hop_selection(self, packet):
        has_route = True
        enquire = False
        self.neighbor_table.purge()

        if self.action_queue is None or self.obs_queue is None:
            best_id = self.neighbor_table.best_neighbor(self.my_drone, packet.dst_drone)
            if best_id == self.my_drone.identifier:
                return False, packet, enquire
            packet.next_hop_id = best_id
            return True, packet, enquire

        context = self._extract_context(packet)
        self.obs_queue.put(("STEP", context))
        action = self.action_queue.get(block=True)
        if action == -1:
            raise simpy.Interrupt("Env reset requested")

        action = int(action)
        if action not in context["valid_neighbors"] or action == self.my_drone.identifier:
            has_route = False
        else:
            packet.next_hop_id = action
            if self.my_drone.identifier not in packet.intermediate_drones:
                packet.intermediate_drones.append(self.my_drone.identifier)
        return has_route, packet, enquire

    def packet_reception(self, packet, src_drone_id):
        current_time = self.simulator.env.now

        if isinstance(packet, self.hello_packet_cls):
            self.neighbor_table.add_item(packet, current_time)
            return

        if isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)
            if packet_copy.dst_drone.identifier == self.my_drone.identifier:
                if packet_copy.packet_id not in self.simulator.metrics.datapacket_arrived:
                    self.simulator.metrics.calculate_metrics(packet_copy)

                config.GL_ID_ACK_PACKET += 1
                ack_packet = AckPacket(
                    src_drone=self.my_drone,
                    dst_drone=self.simulator.drones[src_drone_id],
                    ack_packet_id=config.GL_ID_ACK_PACKET,
                    ack_packet_length=config.ACK_PACKET_LENGTH,
                    ack_packet=packet_copy,
                    simulator=self.simulator,
                    channel_id=packet_copy.channel_id,
                )
                yield self.simulator.env.timeout(config.SIFS_DURATION)
                if not self.my_drone.sleep:
                    ack_packet.increase_ttl()
                    self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                    yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                return

            if self.my_drone.transmitting_queue.qsize() < self.my_drone.max_queue_size:
                self.my_drone.transmitting_queue.put(packet_copy)
                config.GL_ID_ACK_PACKET += 1
                ack_packet = AckPacket(
                    src_drone=self.my_drone,
                    dst_drone=self.simulator.drones[src_drone_id],
                    ack_packet_id=config.GL_ID_ACK_PACKET,
                    ack_packet_length=config.ACK_PACKET_LENGTH,
                    ack_packet=packet_copy,
                    simulator=self.simulator,
                    channel_id=packet_copy.channel_id,
                )
                yield self.simulator.env.timeout(config.SIFS_DURATION)
                if not self.my_drone.sleep:
                    ack_packet.increase_ttl()
                    self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                    yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
            return

        if isinstance(packet, AckPacket):
            data_packet_acked = packet.ack_packet
            if hasattr(data_packet_acked, "first_attempt_time") and data_packet_acked.first_attempt_time is not None:
                self.simulator.metrics.mac_delay.append(
                    (self.simulator.env.now - data_packet_acked.first_attempt_time) / 1e3
                )

            self.my_drone.remove_from_queue(data_packet_acked)
            key = f"wait_ack{self.my_drone.identifier}_{data_packet_acked.packet_id}"
            if self.my_drone.mac_protocol.wait_ack_process_finish.get(key, 1) == 0:
                wait_process = self.my_drone.mac_protocol.wait_ack_process_dict[key]
                if not wait_process.triggered:
                    self.my_drone.mac_protocol.wait_ack_process_finish[key] = 1
                    wait_process.interrupt()
            return

        if isinstance(packet, VfPacket):
            self.my_drone.motion_controller.neighbor_table.add_neighbor(packet, current_time)
            if packet.msg_type == "hello":
                config.GL_ID_VF_PACKET += 1
                channel_id = self.my_drone.channel_assigner.channel_assign()
                ack_packet = VfPacket(
                    src_drone=self.my_drone,
                    creation_time=self.simulator.env.now,
                    id_hello_packet=config.GL_ID_VF_PACKET,
                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                    simulator=self.simulator,
                    channel_id=channel_id,
                )
                ack_packet.msg_type = "ack"
                self.my_drone.transmitting_queue.put(ack_packet)

    def check_waiting_list(self):
        while True:
            if self.action_queue and not self.action_queue.empty() and self.action_queue.queue[0] == -1:
                break
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(self.check_interval)
                for waiting_packet in list(self.my_drone.waiting_list):
                    if self.simulator.env.now > waiting_packet.creation_time + waiting_packet.deadline:
                        self.my_drone.waiting_list.remove(waiting_packet)
                    else:
                        has_route, _, _ = self.next_hop_selection(waiting_packet)
                        if has_route:
                            self.my_drone.transmitting_queue.put(waiting_packet)
                            self.my_drone.waiting_list.remove(waiting_packet)
            else:
                break

    def penalize(self, packet):
        return None
