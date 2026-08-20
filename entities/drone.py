import simpy
import numpy as np
import random
import math
import queue
from simulator.log import logger
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from routing.drl_routing.baseline_drl.baseline_drl import BaselineDrl
from routing.greedy.greedy import Greedy
from routing.grad.grad import Grad
from routing.opar.opar import Opar
from routing.q_routing.q_routing import QRouting
from routing.qfanet.qfanet import QFanet
from routing.qgeo.qgeo import QGeo
from routing.qmr.qmr import QMR
from mac.csma_ca import CsmaCa
from mac.pure_aloha import PureAloha
from mac.tdma import Tdma
from mobility.gauss_markov_3d import GaussMarkov3D
from mobility.random_walk_3d import RandomWalk3D
from mobility.random_waypoint_3d import RandomWaypoint3D
from mobility.trajectory import TraceMobility3D
from energy.energy_model import EnergyModel
from allocation.channel_assignment import ChannelAssigner
from utils import config


class Drone:
    """
    Drone implementation

    Drones in the simulation are served as routers. Each drone can be selected as a potential source node, destination
    and relaying node. Each drone needs to install the corresponding routing module, MAC module, mobility module and
    energy module, etc. At the same time, each drone also has its own queue and can only send one packet at a time, so
    subsequent data packets need queuing for queue resources, which is used to reflect the queue delay in the drone
    network

    Attributes:
        simulator: the simulation platform that contains everything
        env: simulation environment created by simpy
        identifier: used to uniquely represent a drone
        coords: the 3-D position of the drone
        start_coords: the initial position of drone
        direction: current direction of the drone
        pitch: current pitch of the drone
        speed: current speed of the drone
        velocity: velocity components in three directions
        direction_mean: mean direction
        pitch_mean: mean pitch
        velocity_mean: mean velocity
        inbox: a "Store" in simpy, used to receive the packets from other drones (calculate SINR)
        buffer: used to describe the queuing delay of sending packet
        transmitting_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    "transmitting_queue" instead of immediately yield "packet_coming" process. It can prevent the buffer
                    resource of the previous hop node from being occupied all the time
        waiting_list: for reactive routing protocol, if there is no available next hop, it will put the data packet into
                      "waiting_list". Once the routing information bound for a destination is obtained, drone will get
                      the data packets related to this destination, and put them into "transmitting_queue"
        mac_protocol: installed mac protocol (CSMA/CA, ALOHA, etc.)
        mac_process_dict: a dictionary, used to store the mac_process that is launched each time
        mac_process_finish: a dictionary, used to indicate the completion of the process
        mac_process_count: used to distinguish between different "mac_send" processes
        enable_blocking: describe whether the process of waiting for an ACK blocks the delivery of subsequent packets
                         1: stop-and-wait protocol; 0: sliding window (need further implemented)
        routing_protocol: routing protocol installed (GPSR, DSDV, etc.)
        mobility_model: mobility model installed (3-D Gauss-markov, 3-D random waypoint, etc.)
        energy_model: energy consumption model installed
        residual_energy: the residual energy of drone in Joule
        sleep: if the drone is in a "sleep" state, it cannot perform packet sending and receiving operations
        channel_assigner: used to assign sub-channel for transmitting

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/4/16
    """

    def __init__(self,
                 env,
                 node_id,
                 coords,
                 speed,
                 inbox,
                 simulator):
        self.simulator = simulator
        self.env = env
        self.identifier = node_id
        self._coords = [float(value) for value in coords]
        self.start_coords = tuple(self._coords)

        self.rng_drone = random.Random(self.identifier + self.simulator.seed)

        self.direction = self.rng_drone.uniform(0, 2 * np.pi)
        self.pitch = self.rng_drone.uniform(-0.05, 0.05)
        self.speed = speed  # constant speed throughout the simulation
        self.velocity = [self.speed * math.cos(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.pitch)]

        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed

        self.inbox = inbox

        self.buffer = simpy.Resource(env, capacity=1)
        self.max_queue_size = config.MAX_QUEUE_SIZE
        self.transmitting_queue = queue.Queue()  # queue in the real sense
        self.waiting_list = []

        self.mac_protocol = self._create_mac_protocol()
        self.mac_process_dict = dict()
        self.mac_process_finish = dict()
        self.mac_process_count = 0
        self.enable_blocking = 1  # enable "stop-and-wait" protocol

        self.routing_protocol = self._create_routing_protocol()

        self.mobility_model = self._create_mobility_model()
        # self.motion_controller = VfMotionController(self)

        self.energy_model = EnergyModel(self)
        self.residual_energy = config.INITIAL_ENERGY
        self.sleep = False

        self.channel_assigner = ChannelAssigner(self.simulator, self)

        self.env.process(self.generate_data_packet())
        self.env.process(self.feed_packet())
        self.env.process(self.receive())

    @property
    def coords(self):
        return self._coords

    @coords.setter
    def coords(self, position):
        if not self.simulator.airspace.path_is_free(self._coords, position):
            raise ValueError(f"UAV {self.identifier} attempted to enter blocked airspace")
        self._coords = [float(value) for value in position]

    def move_to(self, position, velocity):
        resolved_position, resolved_velocity, collision = self.simulator.airspace.resolve_motion(
            self._coords,
            position,
            velocity,
        )
        self._coords = resolved_position
        self.velocity = resolved_velocity
        if collision is not None:
            self.simulator.event_bus.publish(
                "uav_building_collision",
                self.env.now,
                node=self.identifier,
                building=collision.building_id,
                surface=collision.kind,
                position=list(self._coords),
            )
        return collision

    def _create_mac_protocol(self):
        protocol_map = {
            "CSMA_CA": CsmaCa,
            "ALOHA": PureAloha,
            "PURE_ALOHA": PureAloha,
            "TDMA": Tdma,
        }
        mode = config.MAC_PROTOCOL.replace("-", "_").upper()
        try:
            return protocol_map[mode](self)
        except KeyError as error:
            raise ValueError(f"Unsupported MAC protocol: {config.MAC_PROTOCOL}") from error

    def _create_mobility_model(self):
        if self.simulator.trajectory_trace is not None:
            return TraceMobility3D(self, self.simulator.trajectory_trace)
        model_map = {
            "GAUSSMARKOV3D": GaussMarkov3D,
            "GAUSS_MARKOV_3D": GaussMarkov3D,
            "RANDOMWALK3D": RandomWalk3D,
            "RANDOM_WALK_3D": RandomWalk3D,
            "RANDOMWAYPOINT3D": RandomWaypoint3D,
            "RANDOM_WAYPOINT_3D": RandomWaypoint3D,
        }
        mode = config.MOBILITY_MODEL.replace("-", "_").upper()
        try:
            return model_map[mode](self)
        except KeyError as error:
            raise ValueError(f"Unsupported mobility model: {config.MOBILITY_MODEL}") from error

    def _create_routing_protocol(self):
        routing_mode = getattr(config, "ROUTING_PROTOCOL", "QMR")
        normalized_mode = routing_mode.replace("-", "_").upper()
        protocol_map = {
            "DSDV": Dsdv,
            "GREEDY": Greedy,
            "GRAD": Grad,
            "QROUTING": QRouting,
            "Q_ROUTING": QRouting,
            "QFANET": QFanet,
            "QGEO": QGeo,
            "QMR": QMR,
            "OPAR": Opar,
            "BASELINE_DRL": BaselineDrl,
            "DRL": BaselineDrl,
            "RL": BaselineDrl,
        }

        drl_protocol_cls = getattr(config, "DRL_ROUTING_PROTOCOL_CLASS", None)
        if normalized_mode in {"BASELINE_DRL", "DRL", "RL"} and drl_protocol_cls is not None:
            return drl_protocol_cls(self.simulator, self)

        protocol_cls = protocol_map.get(normalized_mode)
        if protocol_cls is None:
            protocol_cls = drl_protocol_cls
        if protocol_cls is None:
            supported = ", ".join(sorted(protocol_map))
            raise ValueError(f"Unsupported routing protocol '{routing_mode}'. Available: {supported}")
        return protocol_cls(self.simulator, self)

    def generate_data_packet(self):
        """
        Generate one data packet, it should be noted that only when the current packet has been sent can the next
        packet be started. When the drone generates a data packet, it will first put it into the "transmitting_queue",
        the drone reads a data packet from the head of the queue every very short time through "feed_packet()" function.

        """

        traffic_pattern = config.TRAFFIC_PATTERN.upper()
        arrival_rate = float(config.PACKET_ARRIVAL_RATE)
        while True:
            if not self.sleep:
                if traffic_pattern == "UNIFORM":
                    interval_us = 1e6 / arrival_rate
                elif traffic_pattern == "POISSON":
                    interval_us = self.rng_drone.expovariate(arrival_rate) * 1e6
                else:
                    raise ValueError(f"Unsupported traffic pattern: {config.TRAFFIC_PATTERN}")
                yield self.env.timeout(max(1, round(interval_us)))

                config.GL_ID_DATA_PACKET += 1  # data packet id

                # randomly choose a destination
                all_candidate_list = [i for i in range(self.simulator.n_drones)]
                all_candidate_list.remove(self.identifier)
                dst_id = self.rng_drone.choice(all_candidate_list)
                destination = self.simulator.drones[dst_id]  # obtain the destination drone

                # data packet length
                if config.VARIABLE_PAYLOAD_LENGTH:
                    fluctuation = self.rng_drone.randint(-config.MAXIMUM_PAYLOAD_VARIATION, config.MAXIMUM_PAYLOAD_VARIATION)
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH + fluctuation
                else:
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH  # in bit, 1024 bytes

                data_packet_length = (config.IP_HEADER_LENGTH + config.MAC_HEADER_LENGTH +
                                      config.PHY_HEADER_LENGTH + payload_length)

                # channel assignment
                channel_id = self.channel_assigner.channel_assign()

                pkd = DataPacket(self,
                                 dst_drone=destination,
                                 creation_time=self.env.now,
                                 data_packet_id=config.GL_ID_DATA_PACKET,
                                 data_packet_length=data_packet_length,
                                 simulator=self.simulator,
                                 channel_id=channel_id)
                pkd.transmission_mode = 0  # the default transmission mode of data packet is "unicast" (0)

                self.simulator.metrics.record_generated(pkd)

                logger.info('At time: %s (us) ++++ UAV: %s generates a data packet (id: %s, dst: %s)',
                            self.env.now, self.identifier, pkd.packet_id, destination.identifier)

                pkd.waiting_start_time = self.env.now

                if self.transmitting_queue.qsize() < self.max_queue_size:
                    self.transmitting_queue.put(pkd)
                else:
                    self.simulator.event_bus.publish(
                        "packet_dropped",
                        self.env.now,
                        packet_id=pkd.packet_id,
                        node=self.identifier,
                        reason="queue_full",
                    )
            else:  # cannot generate packets if "my_drone" is in sleep state
                break

    def blocking(self):
        """
        The process of waiting for an ACK will block subsequent incoming data packets to simulate the
        "head-of-line blocking problem"
        """

        if self.enable_blocking:
            if not self.mac_protocol.wait_ack_process_finish:
                flag = False  # there is currently no waiting process for ACK
            else:
                # get the latest process status
                final_indicator = list(self.mac_protocol.wait_ack_process_finish.items())[-1]

                if final_indicator[1] == 0:
                    flag = True  # indicates that the drone is still waiting
                else:
                    flag = False  # there is currently no waiting process for ACK
        else:
            flag = False

        return flag

    def feed_packet(self):
        """
        It should be noted that this function is designed for those packets which need to compete for wireless channel

        Firstly, all packets received or generated will be put into the "transmitting_queue", every very short
        time, the drone will read the packet in the head of the "transmitting_queue". Then the drone will check
        if the packet is expired (exceed its maximum lifetime in the network), check the type of packet:
        1) data packet: check if the data packet exceeds its maximum re-transmission attempts. If the above inspection
           passes, routing protocol is executed to determine the next hop drone. If next hop is found, then this data
           packet is ready to transmit, otherwise, it will be put into the "waiting_queue".
        2) control packet: no need to determine next hop, so it will directly start waiting for buffer
        """

        while True:
            if not self.sleep:  # if drone still has enough energy to relay packets
                yield self.env.timeout(10)  # for speed up the simulation

                if not self.blocking():
                    if not self.transmitting_queue.empty():
                        packet = self.transmitting_queue.get()  # get the packet at the head of the queue

                        if self.env.now < packet.creation_time + packet.deadline:  # this packet has not expired
                            if isinstance(packet, DataPacket):
                                if packet.number_retransmission_attempt[self.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                                    # it should be noted that "final_packet" may be the data packet itself or a control
                                    # packet, depending on whether the routing protocol can find an appropriate next hop
                                    has_route, final_packet, enquire = self.routing_protocol.next_hop_selection(packet)

                                    if has_route:
                                        logger.info('At time: %s (us) ---- UAV: %s obtain the next hop: %s of data'
                                                    ' packet (id: %s)',
                                                    self.env.now, self.identifier, packet.next_hop_id, packet.packet_id)

                                        # in this case, the "final_packet" is actually the data packet
                                        yield self.env.process(self.packet_coming(final_packet))
                                    else:
                                        self.waiting_list.append(packet)
                                        self.remove_from_queue(packet)

                                        if enquire:
                                            # in this case, the "final_packet" is actually the control packet
                                            yield self.env.process(self.packet_coming(final_packet))

                            else:  # control packet but not ack
                                yield self.env.process(self.packet_coming(packet))
                        else:
                            self.simulator.event_bus.publish(
                                "packet_dropped",
                                self.env.now,
                                packet_id=packet.packet_id,
                                node=self.identifier,
                                reason="deadline_expired",
                            )
            else:  # this drone runs out of energy
                break  # it is important to break the while loop

    def packet_coming(self, pkd):
        """
        When drone has a packet ready to transmit, yield it.

        The requirement of "ready" is:
            1) this packet is a control packet, or
            2) the valid next hop of this data packet is obtained

        Parameter:
            pkd: packet that waits to enter the buffer of drone
        """

        if not self.sleep:
            arrival_time = self.env.now
            logger.info('At time: %s (us) ---- Packet: %s starts waiting for UAV: %s buffer resource',
                        arrival_time, pkd.packet_id, self.identifier)

            with self.buffer.request() as request:
                yield request  # wait to enter to buffer

                logger.info('At time: %s (us) ---- Packet: %s has been added to the buffer of UAV: %s, '
                            'waiting time is: %s',
                            self.env.now, pkd.packet_id, self.identifier, self.env.now - arrival_time)

                pkd.number_retransmission_attempt[self.identifier] += 1

                if pkd.number_retransmission_attempt[self.identifier] == 1:
                    pkd.time_transmitted_at_last_hop = self.env.now

                logger.info('At time: %s (us) ---- Re-transmission attempts of pkd: %s at UAV: %s is: %s',
                            self.env.now, pkd.packet_id, self.identifier,
                            pkd.number_retransmission_attempt[self.identifier])

                # every time the drone initiates a data packet transmission, "mac_process_count" will be increased by 1
                self.mac_process_count += 1

                key=''.join(['mac_send', str(self.identifier), '_', str(pkd.packet_id)])

                mac_process = self.env.process(self.mac_protocol.mac_send(pkd))
                self.mac_process_dict[key] = mac_process
                self.mac_process_finish[key] = 0

                yield mac_process
        else:
            pass

    def remove_from_queue(self, data_pkd):
        """
        After receiving the ack packet, drone should remove the data packet that has been acked from its queue

        Parameter:
            data_pkd: the acked data packet
        """
        temp_queue = queue.Queue()

        while not self.transmitting_queue.empty():
            pkd_entry = self.transmitting_queue.get()
            if pkd_entry != data_pkd:
                temp_queue.put(pkd_entry)

        while not temp_queue.empty():
            self.transmitting_queue.put(temp_queue.get())

    def receive(self):
        """Pass packets accepted by the physical layer to the routing protocol."""
        while True:
            reception = yield self.inbox.get()
            if self.sleep:
                continue
            packet = reception.packet
            if packet.get_current_ttl() >= self.simulator.n_drones + 1:
                self.simulator.event_bus.publish(
                    "packet_dropped",
                    self.env.now,
                    packet_id=packet.packet_id,
                    node=self.identifier,
                    reason="ttl_exceeded",
                )
                continue
            logger.info(
                "At time: %s (us) ---- Packet %s from UAV: %s is received by UAV: %s, SINR is: %s dB",
                self.env.now,
                packet.packet_id,
                reception.transmitter_id,
                self.identifier,
                reception.sinr_db,
            )
            yield self.env.process(
                self.routing_protocol.packet_reception(packet, reception.transmitter_id)
            )





