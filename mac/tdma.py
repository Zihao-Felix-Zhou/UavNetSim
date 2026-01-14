import simpy
import random
from simulator.log import logger
from phy.phy import Phy
from utils import config
from utils.util_function import check_channel_availability


class Tdma:
    """
    Medium access control protocol: TDMA (Time Division Multiple Access) following IEEE 802.11
    
    The basic flow of TDMA is as follows:
        1) Time is divided into frames, and each frame is divided into time slots
        2) Each node is assigned one or more time slots within a frame
        3) A node can only transmit during its assigned time slot(s)
        4) The node must wait for its slot even if the channel is idle
        5) ACK is sent in the same slot or in a designated ACK period
        
    Main attributes:
        my_drone: the drone that installed the TDMA protocol
        simulator: the simulation platform that contains everything
        rng_mac: a Random class for generating random numbers
        env: simulation environment created by simpy
        phy: the installed physical layer
        channel_states: used to determine if the channel is idle
        enable_ack: use ack or not
        slot_assignment: dictionary mapping drone IDs to their assigned slots
        frame_duration: duration of one TDMA frame
        slot_duration: duration of one time slot
        current_slot: the current slot number in the frame
        
    References:
        [1] IEEE 802.11 Standard for Wireless LAN Medium Access Control (MAC) and 
            Physical Layer (PHY) Specifications
        [2] A. Boukerche, "Algorithms and Protocols for Wireless, Mobile Ad Hoc Networks,"
            Wiley-IEEE Press, 2008.
        [3] C. E. Perkins, "Ad Hoc Networking," Addison-Wesley, 2001.
    
    Author: Based on CSMA/CA implementation
    Created at: 2025/1/13
    """

    def __init__(self, drone):
        self.my_drone = drone
        self.simulator = drone.simulator
        self.rng_mac = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 5)
        self.env = drone.env
        self.phy = Phy(self)
        self.channel_states = self.simulator.channel_states
        self.enable_ack = True

        self.wait_ack_process_dict = dict()
        self.wait_ack_process_finish = dict()
        self.wait_ack_process_count = 0
        self.wait_ack_process = None

        # TDMA-specific attributes
        self.slot_duration = getattr(config, 'TDMA_SLOT_DURATION', 1000)  # default 1000 us
        self.slots_per_frame = getattr(config, 'TDMA_SLOTS_PER_FRAME', 10)  # default 10 slots
        self.frame_duration = self.slot_duration * self.slots_per_frame
        self.guard_time = getattr(config, 'TDMA_GUARD_TIME', 10)  # guard time in us
        
        # Slot assignment - can be static or dynamic
        self.slot_assignment = self._initialize_slot_assignment()
        
        # Start frame synchronization process
        self.env.process(self._frame_sync())

    def _initialize_slot_assignment(self):
        """
        Initialize slot assignments for all drones
        Strategy: Round-robin assignment or can be customized
        :return: dictionary mapping drone_id to list of assigned slot numbers
        """
        slot_assignment = {}
        total_drones = len(self.simulator.drones)
        
        # Simple round-robin assignment
        for i, drone in enumerate(self.simulator.drones):
            slot_assignment[drone.identifier] = [i % self.slots_per_frame]
        
        logger.info('TDMA slot assignment initialized: %s', slot_assignment)
        return slot_assignment

    def _frame_sync(self):
        """
        Maintain frame and slot synchronization
        This process runs continuously to track the current slot
        :return: none
        """
        while True:
            for slot_num in range(self.slots_per_frame):
                self.current_slot = slot_num
                logger.debug('At time: %s (us) ---- Frame slot: %s', self.env.now, slot_num)
                yield self.env.timeout(self.slot_duration)

    def _get_next_slot_start_time(self):
        """
        Calculate the start time of the next assigned slot for this drone
        :return: time to wait until next slot (in us)
        """
        my_slots = self.slot_assignment.get(self.my_drone.identifier, [])
        
        if not my_slots:
            logger.warning('UAV: %s has no assigned slots!', self.my_drone.identifier)
            return self.frame_duration  # wait one full frame
        
        current_time_in_frame = self.env.now % self.frame_duration
        current_slot = int(current_time_in_frame / self.slot_duration)
        
        # Find the next available slot
        for slot in sorted(my_slots):
            slot_start_time = slot * self.slot_duration
            if slot_start_time > current_time_in_frame:
                return slot_start_time - current_time_in_frame
        
        # If no slot found in current frame, get first slot in next frame
        next_slot = sorted(my_slots)[0]
        time_to_frame_end = self.frame_duration - current_time_in_frame
        time_to_next_slot = next_slot * self.slot_duration
        return time_to_frame_end + time_to_next_slot

    def mac_send(self, pkd):
        """
        Control when drone can send packet using TDMA
        :param pkd: the packet that needs to send
        :return: none
        """
        transmission_attempt = pkd.number_retransmission_attempt[self.my_drone.identifier]

        logger.info('At time: %s (us) ---- UAV: %s queues packet: %s for TDMA transmission (attempt: %s)',
                    self.env.now, self.my_drone.identifier, pkd.packet_id, transmission_attempt)

        # Wait for the assigned time slot
        time_to_slot = self._get_next_slot_start_time()
        
        logger.info('At time: %s (us) ---- UAV: %s must wait %s us for its time slot',
                    self.env.now, self.my_drone.identifier, time_to_slot)

        yield self.env.timeout(time_to_slot)

        # Add guard time at the beginning of the slot
        yield self.env.timeout(self.guard_time)

        if pkd.number_retransmission_attempt[self.my_drone.identifier] == 1:
            """
            Record the time when packet first attempts transmission
            """
            pkd.first_attempt_time = self.env.now

        key = ''.join(['mac_send', str(self.my_drone.identifier), '_', str(pkd.packet_id)])
        self.my_drone.mac_process_finish[key] = 1  # mark the process as "finished"

        # Occupy the channel to send packet (in TDMA, collision is avoided by design)
        with self.channel_states[self.my_drone.identifier].request() as req:
            yield req

            logger.info('At time: %s (us) ---- UAV: %s transmits in its TDMA slot (pkd id: %s)',
                        self.env.now, self.my_drone.identifier, pkd.packet_id)

            pkd.transmitting_start_time = self.env.now
            transmission_mode = pkd.transmission_mode

            if transmission_mode == 0:  # for unicast
                next_hop_id = pkd.next_hop_id

                pkd.increase_ttl()
                self.phy.unicast(pkd, next_hop_id)
                yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)  # transmission delay

                logger.info('At time: %s (us) ---- UAV: %s starts to wait ACK for packet: %s',
                            self.env.now, self.my_drone.identifier, pkd.packet_id)

                if self.enable_ack:
                    key2 = ''.join(['wait_ack', str(self.my_drone.identifier), '_', str(pkd.packet_id)])

                    self.wait_ack_process = self.env.process(self.wait_ack(pkd))
                    self.wait_ack_process_dict[key2] = self.wait_ack_process
                    self.wait_ack_process_finish[key2] = 0

                    # Wait for ACK within the same slot (SIFS + ACK transmission time)
                    yield self.env.timeout(config.SIFS_DURATION + config.ACK_PACKET_LENGTH / config.BIT_RATE * 1e6)

            elif transmission_mode == 1:  # for broadcast
                pkd.increase_ttl()
                self.phy.broadcast(pkd)
                yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)

        # Verify we didn't exceed slot duration
        time_in_slot = (self.env.now % self.frame_duration) % self.slot_duration
        if time_in_slot > self.slot_duration - self.guard_time:
            logger.warning('At time: %s (us) ---- UAV: %s transmission exceeded slot boundary!',
                          self.env.now, self.my_drone.identifier)

    def wait_ack(self, pkd):
        """
        If ACK is received within the specified time, the transmission is successful, otherwise,
        a re-transmission will be scheduled for the next available slot
        :param pkd: the data packet that waits for ACK
        :return: none
        """
        try:
            yield self.env.timeout(config.ACK_TIMEOUT)
            self.my_drone.routing_protocol.penalize(pkd)

            logger.info('At time: %s (us) ---- ACK timeout of packet: %s in TDMA slot',
                        self.env.now, pkd.packet_id)

            if pkd.number_retransmission_attempt[self.my_drone.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                # Re-queue packet for transmission in next available slot
                yield self.env.process(self.my_drone.packet_coming(pkd))
            else:
                self.simulator.metrics.mac_delay.append((self.simulator.env.now - pkd.first_attempt_time) / 1e3)

                key2 = ''.join(['wait_ack', str(self.my_drone.identifier), '_', str(pkd.packet_id)])
                self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1

                logger.info('At time: %s (us) ---- Packet: %s is dropped after max retransmissions!',
                            self.env.now, pkd.packet_id)

        except simpy.Interrupt:
            # receive ACK in time
            logger.info('At time: %s (us) ---- UAV: %s receives the ACK for data packet: %s',
                        self.env.now, self.my_drone.identifier, pkd.packet_id)

    def wait_idle_channel(self, sender_drone, drones):
        """
        In TDMA, we don't need to wait for idle channel as slots are pre-assigned
        This method is kept for interface compatibility but does nothing in TDMA
        :param sender_drone: the drone that is about to send packet
        :param drones: a list, which contains all the drones in the simulation
        :return: none
        """
        # In TDMA, channel access is deterministic based on time slots
        # No carrier sensing is needed
        yield self.env.timeout(0)

    def listen(self, channel_states, drones, pkd):
        """
        In TDMA, continuous listening is not required during assigned slots
        This method is kept for interface compatibility but does nothing in TDMA
        :param channel_states: a dictionary, indicates the use of the channel by different drones
        :param drones: a list, contains all drones in the simulation
        :param pkd: the packet being transmitted
        :return: none
        """
        # In TDMA, no listening/carrier sensing is needed
        # Each drone transmits only in its assigned slot
        yield self.env.timeout(0)

    def update_slot_assignment(self, new_assignment):
        """
        Update slot assignments dynamically (for adaptive TDMA)
        :param new_assignment: new slot assignment dictionary
        :return: none
        """
        self.slot_assignment = new_assignment
        logger.info('At time: %s (us) ---- TDMA slot assignment updated: %s',
                    self.env.now, new_assignment)

    def get_slot_utilization(self):
        """
        Calculate the utilization of assigned slots
        :return: utilization percentage
        """
        my_slots = self.slot_assignment.get(self.my_drone.identifier, [])
        return (len(my_slots) / self.slots_per_frame) * 100
