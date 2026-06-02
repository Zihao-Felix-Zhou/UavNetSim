import copy
import random
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator
from simulator.log import logger
from entities.packet import DataPacket
from routing.qfanet.qfanet_packet import QFanetHelloPacket, QFanetAckPacket
from routing.qfanet.qfanet_table import QFanetTable
from utils import config


class QFanet:
    """
    Main procedure of Q-FANET protocol
    References:
        [1] da Costa, Luis Antonio LF, Rafael Kunst, and Edison Pignaton de Freitas.
        "Q-FANET: Improved Q-learning based routing protocol for FANETs." Computer Networks 198 (2021): 108379

    Author: abing, xbb992@vip.qq.com
    Created at: 2025/8/20
    Updated at: 2026/3/10
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.rng_routing = random.Random(my_drone.identifier + simulator.seed + 20)

        self.hello_interval = 0.5 * 1e6
        self.check_interval = 0.6 * 1e6
        self.learning_rate = 0.6  # fixed alpha
        self.discount_factor = 1.0  # fixed discount factor
        self.lookback = 10  # history window size
        self.sinr_weight = 0.7  # SINR weight

        self.eps = 0.9
        self.r_max = 100
        self.r_min = -100
        self.r_default = 50

        self.table = QFanetTable(simulator.env, my_drone, self.rng_routing)

        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())
        self.env = simulator.env
    def broadcast_hello_packet(self):
        """Generate Hello packet"""
        config.GL_ID_HELLO_PACKET += 1
        channel_id = self.my_drone.channel_assigner.channel_assign()

        hello_pkd = QFanetHelloPacket(
            src_drone=self.my_drone,
            creation_time=self.simulator.env.now,
            id_hello_packet=config.GL_ID_HELLO_PACKET,
            hello_packet_length=config.HELLO_PACKET_LENGTH,
            simulator=self.simulator,
            channel_id=channel_id
        )
        hello_pkd.transmission_mode = 1
        logger.info(
            'At time: %s (us) ---- UAV: %s broadcasts Q-FANET HELLO (ID: %s)',
            self.simulator.env.now, self.my_drone.identifier, hello_pkd.packet_id
        )
        self.simulator.metrics.control_packet_num += 1
        self.my_drone.transmitting_queue.put(hello_pkd)

    def broadcast_hello_packet_periodically(self):
        """Broadcast hello packet periodically"""
        while True:
            self.broadcast_hello_packet()
            jitter = self.rng_routing.randint(1000, 2000)
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
        self.table.purge()
        dst_drone = packet.dst_drone
        if self.my_drone.identifier not in packet.intermediate_drones:
            packet.intermediate_drones.append(self.my_drone.identifier)

        best_next_hop_id = self.table.best_neighbor(packet, dst_drone, self.eps)
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
        src_drone = self.simulator.drones[src_drone_id]

        if isinstance(packet, QFanetHelloPacket):
            packet_copy = copy.copy(packet)
            current_sinr = self.cal_p2p_sinr(packet_copy, src_drone_id)
            self.table.add_item(packet_copy, current_time, current_sinr)
            logger.info(
                'At time: %s (us) ---- UAV: %s receives HELLO from %s (SINR: %.1f dB)',
                current_time, self.my_drone.identifier, src_drone_id, current_sinr
            )
        elif isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)
            dst_id = packet_copy.dst_drone.identifier
            sinr = self.cal_p2p_sinr(packet_copy, src_drone_id)
            sinr_eta = self.table.calculate_eta(sinr)  # sinr => eta
            ack_packet = None
            if dst_id == self.my_drone.identifier:
                if packet_copy.packet_id not in self.simulator.metrics.datapacket_arrived:
                    self.simulator.metrics.calculate_metrics(packet_copy)

                config.GL_ID_ACK_PACKET += 1
                src_drone = self.simulator.drones[src_drone_id]  # previous drone
                ack_packet = QFanetAckPacket(
                    src_drone=self.my_drone,
                    dst_drone=src_drone,
                    ack_packet_id=config.GL_ID_ACK_PACKET,
                    ack_packet_length=config.ACK_PACKET_LENGTH,
                    acked_packet=packet_copy,
                    void_area_flag=self.table.void_area_judgment(packet_copy.dst_drone),
                    reward=self.r_max,
                    sinr_eta=sinr_eta,
                    simulator=self.simulator,
                    channel_id=packet_copy.channel_id,
                    creation_time=self.simulator.env.now
                )

            else:
                if self.my_drone.transmitting_queue.qsize() < self.my_drone.max_queue_size:
                    self.my_drone.transmitting_queue.put(packet_copy)
                    void_flag = self.table.void_area_judgment(packet_copy.dst_drone)
                    reward = self.r_min if void_flag else self.r_default

                    config.GL_ID_ACK_PACKET += 1
                    ack_packet = QFanetAckPacket(
                        src_drone=self.my_drone,
                        dst_drone=src_drone,
                        ack_packet_id=config.GL_ID_ACK_PACKET,
                        ack_packet_length=config.ACK_PACKET_LENGTH,
                        acked_packet=packet_copy,
                        void_area_flag=void_flag,
                        reward=reward,
                        sinr_eta=sinr_eta,
                        simulator=self.simulator,
                        channel_id=packet_copy.channel_id,
                        creation_time=self.simulator.env.now
                    )
            if ack_packet is not None:
                yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                # unicast the ack packet immediately without contention for the channel
                if not self.my_drone.sleep:
                    ack_packet.increase_ttl()
                    self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                    yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                    self.simulator.drones[src_drone_id].receive()
                else:
                    pass

        elif isinstance(packet, QFanetAckPacket):
            data_packet_acked = packet.acked_packet
            dst_id = packet.acked_packet.dst_drone.identifier
            mac_delay = current_time - data_packet_acked.first_attempt_time
            self.simulator.metrics.mac_delay.append(mac_delay / 1e3)
            self.table.update_q_value(
                next_hop_id=src_drone_id,
                dst_id=dst_id,
                reward=packet.reward,
                sinr_eta=packet.sinr_eta
            )
            self.my_drone.remove_from_queue(data_packet_acked)

            key2 = 'wait_ack' + str(self.my_drone.identifier) + '_' + str(data_packet_acked.packet_id)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                if not self.my_drone.mac_protocol.wait_ack_process_dict[key2].triggered:
                    logger.info('At time: %s (us) ---- wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                                self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)

                    self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1  # mark it as "finished"
                    self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()

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

    def cal_p2p_sinr(self, data_packet, previous_drone_id):
        """Calculate point to point sinr"""
        main_drones_list = [[previous_drone_id, data_packet.channel_id]]
        all_transmitting = self.get_current_transmitting_nodes()
        sinr_list = sinr_calculator(self.my_drone, main_drones_list, all_transmitting)
        current_sinr = sinr_list[0]  # only one main transmitter
        return current_sinr

    def penalize(self, packet):
        dst_id = packet.dst_drone.identifier
        next_hop_id = packet.next_hop_id

        self.table.update_q_value(
            next_hop_id=next_hop_id,
            dst_id=dst_id,
            reward=self.r_min,
            sinr_eta=0
        )

    def get_current_transmitting_nodes(self):
        """Get all the nodes that are currently transmitting"""
        transmitting_nodes = []
        for drone in self.simulator.drones:
            for item in drone.inbox:
                packet = item[0]
                insertion_time = item[1]
                transmitter = item[2]
                channel_used = item[4]
                transmitting_time = packet.packet_length / config.BIT_RATE * 1e6
                interval = [insertion_time, insertion_time + transmitting_time]
                if has_intersection(interval, [self.env.now, self.env.now]):
                    transmitting_nodes.append([transmitter, channel_used])
        return [list(x) for x in {tuple(i) for i in transmitting_nodes}]