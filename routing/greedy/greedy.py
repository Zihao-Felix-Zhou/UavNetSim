import copy
import random
from simulator.log import logger
from entities.packet import DataPacket, AckPacket
from topology.virtual_force.vf_packet import VfPacket
from routing.greedy.greedy_neighbor_table import GreedyNeighborTable
from routing.greedy.greedy_packet import GreedyHelloPacket
from routing.parameters import routing_interval_us
from utils import config


class Greedy:
    """
    Main procedure of Greedy Forwarding

    In greedy forwarding approach, packets are forwarded to the neighbor located closest to the destination at each hop.
    Greedy routing is highly efficient for the route discovery process, however, the problem of local minimum will
    significantly limit its performance. Users can extend it by adding some recovery mechanism to improve the
    performance.

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the greedy routing
        rng_routing: a Random class based on which we can call the function that generates the random number
        hello_interval: interval of sending hello packet
        neighbor_table: neighbor table of greedy routing

    References:
        [1] N. K. Gupta, R. S. Yadav and R. K. Nagaria, "3D geographical routing protocols in wireless ad hoc and sensor
            networks: An overview," in Wireless Networks, vol. 26, pp. 2549-2566, 2020.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/4/15
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.rng_routing = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 10)
        self.hello_interval = routing_interval_us("hello_interval_s", 0.5)
        self.check_interval = 0.6 * 1e6
        self.neighbor_table = GreedyNeighborTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())

    def broadcast_hello_packet(self, my_drone):
        config.GL_ID_HELLO_PACKET += 1

        # channel assignment
        channel_id = self.my_drone.channel_assigner.channel_assign()

        hello_pkd = GreedyHelloPacket(src_drone=my_drone,
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
            next hop drone id
        """

        has_route = True
        enquire = False  # "True" when reactive protocol is adopted

        # update neighbor table
        self.neighbor_table.purge()

        dst_drone = packet.dst_drone

        # choose best next hop according to the neighbor table
        best_next_hop_id = self.neighbor_table.best_neighbor(self.my_drone, dst_drone)

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
        if isinstance(packet, GreedyHelloPacket):
            self.neighbor_table.add_item(packet, current_time)  # update the neighbor table
            self.neighbor_table.print_item(self.my_drone)

        elif isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)

            if packet_copy.dst_drone.identifier == self.my_drone.identifier:
                if packet_copy.packet_id not in self.simulator.metrics.datapacket_arrived:
                    self.simulator.metrics.calculate_metrics(packet_copy)

                    logger.info('At time: %s (us) ---- Data packet: %s is received by destination UAV: %s',
                                self.simulator.env.now, packet_copy.packet_id, self.my_drone.identifier)

                # reply ACK
                config.GL_ID_ACK_PACKET += 1
                src_drone = self.simulator.drones[src_drone_id]  # previous drone

                # NOTE: The pair of transceivers for a particular link are tuned to the same channel for transmission
                # in either direction (i.e., there is no directionality in channel assignment).
                ack_packet = AckPacket(src_drone=self.my_drone,
                                       dst_drone=src_drone,
                                       ack_packet_id=config.GL_ID_ACK_PACKET,
                                       ack_packet_length=config.ACK_PACKET_LENGTH,
                                       ack_packet=packet_copy,
                                       simulator=self.simulator,
                                       channel_id=packet_copy.channel_id)

                yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                # unicast the ack packet immediately without contention for the channel
                if not self.my_drone.sleep:
                    ack_packet.increase_ttl()
                    self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                    yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                else:
                    pass
            else:
                if self.my_drone.transmitting_queue.qsize() < self.my_drone.max_queue_size:  # have enough capacity
                    logger.info('At time: %s (us) ---- Data packet: %s is received by next hop UAV: %s',
                                self.simulator.env.now, packet_copy.packet_id, self.my_drone.identifier)

                    self.my_drone.transmitting_queue.put(packet_copy)  # add this packet into my own queue

                    config.GL_ID_ACK_PACKET += 1
                    src_drone = self.simulator.drones[src_drone_id]  # previous drone
                    ack_packet = AckPacket(src_drone=self.my_drone,
                                           dst_drone=src_drone,
                                           ack_packet_id=config.GL_ID_ACK_PACKET,
                                           ack_packet_length=config.ACK_PACKET_LENGTH,
                                           ack_packet=packet_copy,
                                           simulator=self.simulator,
                                           channel_id=packet_copy.channel_id)

                    yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                    # unicast the ack packet immediately without contention for the channel
                    if not self.my_drone.sleep:
                        ack_packet.increase_ttl()
                        self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                        yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                    else:
                        pass
                else:  # the queue is full, discard this packet and no ACK reply
                    pass

        elif isinstance(packet, AckPacket):
            data_packet_acked = packet.ack_packet

            self.simulator.metrics.mac_delay.append(
                (self.simulator.env.now - data_packet_acked.first_attempt_time) / 1e3)

            self.my_drone.remove_from_queue(data_packet_acked)

            key2 = ''.join(['wait_ack', str(self.my_drone.identifier), '_', str(data_packet_acked.packet_id)])

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

                # channel assignment
                channel_id = self.my_drone.channel_assigner.channel_assign()

                ack_packet = VfPacket(src_drone=self.my_drone,
                                      creation_time=self.simulator.env.now,
                                      id_hello_packet=config.GL_ID_VF_PACKET,
                                      hello_packet_length=config.HELLO_PACKET_LENGTH,
                                      simulator=self.simulator,
                                      channel_id=channel_id)
                ack_packet.msg_type = 'ack'

                self.my_drone.transmitting_queue.put(ack_packet)
            else:
                pass

    def check_waiting_list(self):
        while True:
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(self.check_interval)
                for waiting_pkd in list(self.my_drone.waiting_list):
                    if self.simulator.env.now > waiting_pkd.creation_time + waiting_pkd.deadline:
                        self.my_drone.waiting_list.remove(waiting_pkd)
                    else:
                        dst_drone = waiting_pkd.dst_drone
                        best_next_hop_id = self.neighbor_table.best_neighbor(self.my_drone, dst_drone)
                        if best_next_hop_id != self.my_drone.identifier:
                            self.my_drone.transmitting_queue.put(waiting_pkd)
                            self.my_drone.waiting_list.remove(waiting_pkd)
                        else:
                            pass
            else:
                break

    def penalize(self, packet):
        pass
