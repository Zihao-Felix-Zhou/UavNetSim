import copy
import random
from simulator.log import logger
from entities.packet import DataPacket
from routing.qmr import qmr_config
from routing.qmr.history_packets_recorder import HistoryPacketsRecorder
from routing.qmr.qmr_table import QMRTable
from routing.qmr.qmr_packet import QMRHelloPacket, QMRAckPacket
from utils import config


class QMR:
    """
    Main procedure of QMR: Q-learning based Multi-objective optimization routing protocol

    References:
        [1] J. Liu, Q. Wang, C. He, K. Jaffrès-Runser, Y. Xu, Z. Li and Y. Xu, "QMR:Q-learning based Multi-objective
        Optimization Routing protocol for Flying Ad Hoc Networks," Computer Communications, vol. 150, pp. 304-316, 2020.

    Created at: 2025/2/9
    Updated at: 2025/7/2
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone

        self.table = QMRTable(simulator.env, my_drone)
        self.history_packet_recorder = HistoryPacketsRecorder(self.simulator.n_drones)
        self.rng_routing = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 10)

        self.eps = 0.8  # epsilon-greedy
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet periodically

        self.first_hello = True

        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())
        self.simulator.env.process(self.update_discounted_factor())
        self.simulator.env.process(self.update_eps())

    def broadcast_hello_packet(self, my_drone):
        """Generate one hello packet"""

        config.GL_ID_HELLO_PACKET += 1

        # channel assignment
        channel_id = self.my_drone.channel_assigner.channel_assign()

        cur_time = self.simulator.env.now
        received_hello_packet_tuple = self.history_packet_recorder.get_all_active_received_hello_packet_count(cur_time)

        hello_pkt = QMRHelloPacket(
            src_drone=my_drone,
            creation_time=cur_time,
            id_hello_packet=config.GL_ID_HELLO_PACKET,
            hello_packet_length=config.HELLO_PACKET_LENGTH,
            received_hello_packet_count=received_hello_packet_tuple,
            simulator=self.simulator,
            channel_id=channel_id
        )
        hello_pkt.transmission_mode = 1

        logger.info('At time: %s (us) ---- UAV: %s has a hello packet to broadcast',
                    self.simulator.env.now, self.my_drone.identifier)

        self.history_packet_recorder.add_sent_hello_packet(hello_pkt)

        self.simulator.metrics.control_packet_num += 1
        self.my_drone.transmitting_queue.put(hello_pkt)

    def broadcast_hello_packet_periodically(self):
        """Broadcast hello packet periodically"""

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

        packet.intermediate_drones.append(self.my_drone.identifier)
        next_hop_id = self.table.make_route_decision(packet, dst_drone, self.eps)

        if next_hop_id is self.my_drone.identifier:
            has_route = False  # no available next hop
        else:
            packet.next_hop_id = next_hop_id  # it has an available next hop drone
        
        if has_route:
            self.history_packet_recorder.add_sent_data_packet(packet)

        return has_route, packet, enquire

    def update_discounted_factor(self):
        while True:
            self.table.update_discounted_factor()
            yield self.simulator.env.timeout(qmr_config.discount_factor_update_interval)

    def packet_reception(self, packet, src_drone_id):
        """
        Packet reception at network layer

        since different routing protocols have their own corresponding packets, it is necessary to add this packet
        reception function in the network layer

        Parameters:
            packet: the received packet
            src_drone_id: previous hop
        """

        cur_time = self.simulator.env.now

        if isinstance(packet, QMRHelloPacket):
            self.table.update_neighbor(packet, cur_time)
            self.history_packet_recorder.add_received_hello_packet(packet)

        elif isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)
            packet_copy.previous_drone = self.simulator.drones[src_drone_id]
            queuing_delay = packet_copy.transmitting_start_time - packet_copy.waiting_start_time
            
            
            config.GL_ID_ACK_PACKET += 1
            src_drone = self.simulator.drones[src_drone_id]
            max_q = self.table.get_max_q()

            is_local_minimum = self.table.check_local_minimum(packet.dst_drone)

            ack_packet = QMRAckPacket(
                creation_time=cur_time,
                src_drone=self.my_drone,
                dst_drone=src_drone,
                ack_packet_id=config.GL_ID_ACK_PACKET,
                ack_packet_length=config.ACK_PACKET_LENGTH,
                ack_packet=packet_copy,
                transmitting_start_time=cur_time,
                queuing_delay=queuing_delay,
                max_q=max_q,
                is_local_minimum=is_local_minimum,
                source_packet_backoff_start_time=packet.first_attempt_time,
                simulator=self.simulator,
                channel_id=packet_copy.channel_id
            )
            yield self.simulator.env.timeout(config.SIFS_DURATION)

            if not self.my_drone.sleep:
                ack_packet.increase_ttl()
                self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                self.simulator.drones[src_drone_id].receive()
            else:
                pass

            if packet_copy.dst_drone.identifier == self.my_drone.identifier:
                if packet_copy.packet_id not in self.simulator.metrics.datapacket_arrived:
                    self.simulator.metrics.calculate_metrics(packet_copy)
            else:
                if self.my_drone.transmitting_queue.qsize() < config.MAX_QUEUE_SIZE:
                    logger.info('At time: %s (us) ---- Data packet: %s is received by next hop UAV: %s',
                                self.simulator.env.now, packet_copy.packet_id, self.my_drone.identifier)

                    self.my_drone.transmitting_queue.put(packet_copy)
                else:
                    pass

        elif isinstance(packet, QMRAckPacket): 
            original_packet = packet.ack_packet

            cur_time = self.simulator.env.now
            mac_delay = cur_time - packet.source_packet_backoff_start_time
            self.simulator.metrics.mac_delay.append(mac_delay / 1e3)
            self.table.add_mac_delay(mac_delay, cur_time, packet.src_drone.identifier)

            self.history_packet_recorder.add_received_ack_packet(packet)

            self.update(packet, src_drone_id)

            key2 = f"wait_ack{self.my_drone.identifier}_{original_packet.packet_id}"

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                if not self.my_drone.mac_protocol.wait_ack_process_dict[key2].triggered:
                    logger.info('At time: %s, the wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                                self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)
                    self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1  # marked it as finished
                    self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()

    def update(self, packet, next_hop_id):
        origin_data_packet = packet.ack_packet
        dst_drone = origin_data_packet.dst_drone

        max_q = packet.max_q

        if next_hop_id == dst_drone.identifier:
            f = 1
        else:
            f = 0

        self.table.update_q_value(f, max_q, next_hop_id, packet.is_local_minimum, dst_drone)

    def check_waiting_list(self):
        while True:
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(0.6 * 1e6)
                for waiting_pkd in list(self.my_drone.waiting_list):
                    if self.simulator.env.now > waiting_pkd.creation_time + waiting_pkd.deadline:
                        self.my_drone.waiting_list.remove(waiting_pkd)
                    else:
                        dst_drone = waiting_pkd.dst_drone
                        best_next_hop_id = self.table.make_route_decision(waiting_pkd, dst_drone, self.eps)
                        if best_next_hop_id != self.my_drone.identifier:
                            self.my_drone.transmitting_queue.put(waiting_pkd)
                            self.my_drone.waiting_list.remove(waiting_pkd)
                        else:
                            pass
            else:
                break

    def penalty_for_ack_loss(self, packet):
        next_hop_id = packet.next_hop_id
        f = 1 if next_hop_id == packet.dst_drone.identifier else 0

        # TODO: check this out
        q_max = self.table.get_last_max_q_value_of_neighbor(next_hop_id)
        is_penalty = True
        self.table.update_q_value(f, q_max, next_hop_id, is_penalty)

    def update_eps(self):
        while True:
            yield self.simulator.env.timeout(self.hello_interval)
            self.eps = max(qmr_config.eps_decay * self.eps, 0.2)

    def penalize(self, packet):
        pass
