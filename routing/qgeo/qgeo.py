import copy
import math
import random
from simulator.log import logger
from entities.packet import DataPacket
from topology.virtual_force.vf_packet import VfPacket
from routing.qgeo.qgeo_packet import QGeoHelloPacket, QGeoAckPacket
from routing.qgeo.qgeo_table import QGeoTable
from utils import config
from utils import util_function
from phy.large_scale_fading import maximum_communication_range


class QGeo:
    """
    Main procedure of QGeo (without consideration of link error and location error)

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the GPSR
        rng_routing: a Random class based on which we can call the function that generates the random number
        hello_interval: time interval of sending hello packet
        check_interval: time interval of checking the waiting list
        learning_rate: hyperparameter in Q-learning
        r_max: if the next hop is the destination, the maximum reward "r_max" will be given
        r_min: if void area is reached or no ACK is received, the minimum reward "r_min" will be given
        table: including neighbor table and Q-table

    References:
        [1] W. Jung, J. Yim and Y. Ko, "QGeo: Q-learning-based Geographic Ad Hoc Routing Protocol for Unmanned Robotic
            Networks," in IEEE Communications Letters, vol. 21, no. 10, pp. 2258-2261, 2017.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2025/2/22
    Updated at: 2025/4/15
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.rng_routing = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 10)
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet periodically
        self.check_interval = 0.6 * 1e6
        self.learning_rate = 0.6  # fixed learning rate
        self.r_max = 10
        self.r_min = -self.r_max
        self.table = QGeoTable(self.simulator.env, my_drone,self.rng_routing)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())

    def broadcast_hello_packet(self, my_drone):
        config.GL_ID_HELLO_PACKET += 1

        # channel assignment
        channel_id = self.my_drone.channel_assigner.channel_assign()

        hello_pkd = QGeoHelloPacket(src_drone=my_drone,
                                    creation_time=self.simulator.env.now,
                                    id_hello_packet=config.GL_ID_HELLO_PACKET,
                                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                                    simulator=self.simulator,
                                    channel_id=channel_id)
        hello_pkd.transmission_mode = 1

        logger.info('At time: %s (us) ---- UAV: %s has a hello packet to broadcast',
                    self.simulator.env.now, self.my_drone.identifier)

        self.simulator.metrics.control_packet_num += 1
        self.my_drone.transmitting_queue.put(hello_pkd)

    def broadcast_hello_packet_periodically(self):
        while True:
            self.broadcast_hello_packet(self.my_drone)
            jitter = self.rng_routing.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval + jitter)

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing protocol

        Parameters:
            packet: the data packet that needs to be sent

        Returns:
            Next hop drone
        """
        enquire = False
        has_route = True

        # update neighbor table
        self.table.purge()

        dst_drone = packet.dst_drone

        # choose best next hop according to the neighbor table
        packet.intermediate_drones.append(self.my_drone.identifier)

        best_next_hop_id = self.table.best_neighbor(self.my_drone, dst_drone)

        if best_next_hop_id is self.my_drone.identifier:
            has_route = False  # no available next hop
        else:
            packet.next_hop_id = best_next_hop_id  # it has an available next hop drone

        return has_route, packet, enquire

    def packet_reception(self, packet, src_drone_id):
        """
        Packet reception at network layer

        since different routing protocols have their own corresponding packets, it is necessary to add this packet
        reception function in the network layer

        Parameters:
            packet: the received packet
            src_drone_id: previous hop
        """

        current_time = self.simulator.env.now
        if isinstance(packet, QGeoHelloPacket):
            self.table.add_item(packet, current_time)  # update the neighbor table

        elif isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)

            packet_copy.previous_drone = self.simulator.drones[src_drone_id]

            if packet_copy.dst_drone.identifier == self.my_drone.identifier:
                if packet_copy.packet_id not in self.simulator.metrics.datapacket_arrived:
                    self.simulator.metrics.calculate_metrics(packet_copy)

                    logger.info('At time: %s (us) ---- Data packet: %s is received by destination UAV: %s',
                                self.simulator.env.now, packet_copy.packet_id, self.my_drone.identifier)

                config.GL_ID_ACK_PACKET += 1
                src_drone = self.simulator.drones[src_drone_id]  # previous drone

                reward = self.r_max
                max_q = self.table.get_max_q_value(packet_copy.dst_drone.identifier)

                ack_packet = QGeoAckPacket(src_drone=self.my_drone,
                                           dst_drone=src_drone,
                                           ack_packet_id=config.GL_ID_ACK_PACKET,
                                           ack_packet_length=config.ACK_PACKET_LENGTH,
                                           acked_packet=packet,
                                           void_area_flag=0,
                                           reward=reward,
                                           max_q=max_q,
                                           simulator=self.simulator,
                                           channel_id=packet_copy.channel_id)

                yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                # unicast the ack packet immediately without contention for the channel
                if not self.my_drone.sleep:
                    ack_packet.increase_ttl()
                    self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                    yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                    self.simulator.drones[src_drone_id].receive()
                else:
                    pass
            else:
                if self.my_drone.transmitting_queue.qsize() < self.my_drone.max_queue_size:
                    logger.info('At time: %s (us) ---- Data packet: %s is received by next hop UAV: %s',
                                self.simulator.env.now, packet_copy.packet_id, self.my_drone.identifier)

                    self.my_drone.transmitting_queue.put(packet_copy)

                    config.GL_ID_ACK_PACKET += 1
                    src_drone = self.simulator.drones[src_drone_id]  # previous drone

                    void_area_flag = self.table.void_area_judgment(packet_copy.dst_drone)

                    if void_area_flag == 0:
                        distance1 = util_function.euclidean_distance_3d(self.simulator.drones[src_drone_id].coords,
                                                                        packet_copy.dst_drone.coords)
                        distance2 = util_function.euclidean_distance_3d(self.my_drone.coords,
                                                                        packet_copy.dst_drone.coords)
                        reward = (distance1 - distance2) / maximum_communication_range()
                        max_q = self.table.get_max_q_value(packet_copy.dst_drone.identifier)
                    else:
                        reward = self.r_min
                        max_q = 0

                    ack_packet = QGeoAckPacket(src_drone=self.my_drone,
                                               dst_drone=src_drone,
                                               ack_packet_id=config.GL_ID_ACK_PACKET,
                                               ack_packet_length=config.ACK_PACKET_LENGTH,
                                               acked_packet=packet,
                                               void_area_flag=void_area_flag,
                                               reward=reward,
                                               max_q=max_q,
                                               simulator=self.simulator,
                                               channel_id=packet_copy.channel_id)

                    yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                    # unicast the ack packet immediately without contention for the channel
                    if not self.my_drone.sleep:
                        ack_packet.increase_ttl()
                        self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                        yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                        self.simulator.drones[src_drone_id].receive()
                    else:
                        pass
                else:
                    pass

        elif isinstance(packet, QGeoAckPacket):
            data_packet_acked = packet.acked_packet
            next_hop_coords = packet.src_coords
            next_hop_velocity = packet.src_velocity

            # update Q-table
            self.update_q_table(packet, src_drone_id, next_hop_coords, next_hop_velocity)

            self.my_drone.remove_from_queue(data_packet_acked)

            key2 = 'wait_ack' + str(self.my_drone.identifier) + '_' + str(data_packet_acked.packet_id)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                if not self.my_drone.mac_protocol.wait_ack_process_dict[key2].triggered:
                    logger.info('At time: %s (us) ---- wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                                self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)

                    self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1  # mark it as "finished"
                    self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()

        elif isinstance(packet, VfPacket):
            logger.info('At time: %s (us) ---- UAV: %s receives the vf hello msg from UAV: %s, pkd id is: %s',
                        self.simulator.env.now, self.my_drone.identifier, src_drone_id, packet.packet_id)

            # update the neighbor table
            self.my_drone.motion_controller.neighbor_table.add_neighbor(packet, current_time)

            if packet.msg_type == 'hello':
                config.GL_ID_VF_PACKET += 1

                ack_packet = VfPacket(src_drone=self.my_drone,
                                      creation_time=self.simulator.env.now,
                                      id_hello_packet=config.GL_ID_VF_PACKET,
                                      hello_packet_length=config.HELLO_PACKET_LENGTH,
                                      simulator=self.simulator,
                                      channel_id=packet.channel_id)
                ack_packet.msg_type = 'ack'

                self.my_drone.transmitting_queue.put(ack_packet)
            else:
                pass

    def update_q_table(self, packet, next_hop_id, next_hop_coords, next_hop_velocity):
        data_packet_acked = packet.acked_packet
        dst_drone = data_packet_acked.dst_drone

        self.simulator.metrics.mac_delay.append((self.simulator.env.now - data_packet_acked.first_attempt_time) / 1e3)

        reward = packet.reward
        max_q = packet.max_q

        # calculate reward function
        if next_hop_id == dst_drone.identifier:
            f = 1
        else:
            f = 0

        # calculate the discounted factor
        t = (math.ceil(self.simulator.env.now / self.hello_interval) *
             self.hello_interval - self.simulator.env.now) / 1e6
        future_pos_myself = self.my_drone.coords + [i * t for i in self.my_drone.velocity]
        future_pos_next_hop = next_hop_coords + [i * t for i in next_hop_velocity]
        future_distance = util_function.euclidean_distance_3d(future_pos_myself, future_pos_next_hop)

        if future_distance < maximum_communication_range():
            gamma = 0.6
        else:
            gamma = 0.4

        self.table.q_table[next_hop_id][dst_drone.identifier] = \
            (1 - self.learning_rate) * self.table.q_table[next_hop_id][dst_drone.identifier] + \
            self.learning_rate * (reward + gamma * (1 - f) * max_q)

    def check_waiting_list(self):
        while True:
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(self.check_interval)
                for waiting_pkd in list(self.my_drone.waiting_list):
                    if self.simulator.env.now > waiting_pkd.creation_time + waiting_pkd.deadline:
                        self.my_drone.waiting_list.remove(waiting_pkd)
                    else:
                        has_route, packet, enquire = self.next_hop_selection(waiting_pkd)
                        if has_route:
                            self.my_drone.transmitting_queue.put(waiting_pkd)
                            self.my_drone.waiting_list.remove(waiting_pkd)
                        else:
                            pass
            else:
                break

    def penalize(self, packet):
        dst_id = packet.dst_drone.identifier
        next_hop_id = packet.next_hop_id

        reward = self.r_min

        self.table.q_table[next_hop_id][dst_id] = \
            (1 - self.learning_rate) * self.table.q_table[next_hop_id][dst_id] + \
            self.learning_rate * reward
