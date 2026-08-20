import simpy
import random
from simulator.log import logger
from phy.phy import Phy
from utils import config


class CsmaCa:
    """
    Medium access control protocol: CSMA/CA (Carrier Sense Multiple Access With Collision Avoidance) without RTS/CTS

    The basic flow of the basic CSMA/CA (without RTS/CTS) is as follows:
        1) when a node has a packet to send, it first needs to wait until the channel is idle
        2) when the channel is idle, the node starts a timer and waits for "DIFS+backoff" periods of time, where the
           length of backoff is related to the number of re-transmissions
        3) if the entire decrement of the timer to 0 is not interrupted, then the node can occupy the channel and start
           sending the data packet and waiting for the corresponding ACK
        4) if the countdown is interrupted, it means that the node loses the game. The node should freeze the timer and
           wait for channel idle again before re-starting its timer

    Main attributes:
        my_drone: the drone that installed the CSMA/CA protocol
        simulator: the simulation platform that contains everything
        rng_mac: a Random class based on which we can call the function that generates the random number
        env: simulation environment created by simpy
        phy: the installed physical layer
        channel_states: used to determine if the channel is idle
        enable_ack: use ack or not

    References:
        [1] J. Li, et al., "Packet Delay in UAV Wireless Networks Under Non-saturated Traffic and Channel Fading
            Conditions," Wireless Personal Communications, vol. 72, no. 2, pp. 1105-1123, 2013.
        [2] M. A. Siddique and J. Kamruzzaman, "Performance Analysis of M-Retry BEB Based DCF Under Unsaturated Traffic
            Condition," in 2010 IEEE Wireless Communication and Networking Conference, 2010, pp. 1-6.
        [3] F. Daneshgaran, M. Laddomada, F. Mesiti and M. Mondin, "Unsaturated Throughput Analysis of IEEE 802.11 in
            Presence of Non-ideal Transmission Channel and Capture Effects," IEEE transactions on Wireless Communications
            vol. 7, no. 4, pp. 1276-1286, 2008.
        [4] Park, Ki hong, "Wireless Lecture Notes". Purdue. Retrieved 28 September 2012, link:
            https://www.cs.purdue.edu/homes/park/cs536-wireless-3.pdf

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/4/15
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

    def mac_send(self, pkd):
        """
        Control when drone can send packet
        :param pkd: the packet that needs to send
        :return: none
        """

        transmission_attempt = pkd.number_retransmission_attempt[self.my_drone.identifier]
        contention_window = (config.CW_MIN + 1) * (2 ** (transmission_attempt-1)) - 1

        backoff = self.rng_mac.randint(0, contention_window - 1) * config.SLOT_DURATION  # random backoff, in us
        to_wait = config.DIFS_DURATION + backoff

        logger.info('At time: %s (us) ---- UAV: %s sets its back-off counter as: %s',
                    self.env.now, self.my_drone.identifier, backoff)

        while to_wait:
            # wait until the channel becomes idle
            yield self.env.process(self.wait_idle_channel(pkd.channel_id))

            if pkd.number_retransmission_attempt[self.my_drone.identifier] == 1:
                """
                NOTE: because the service time of the packet is given by the interval between the time when this packet
                starts backoff and the time when it is acknowledged and removed from the queue, so the "backoff_start_
                time" should be recorded only when the drone transmits this packet for the first time.
                """
                pkd.first_attempt_time = self.env.now

            # start listen the channel at backoff stage
            self.env.process(self.listen(pkd))

            logger.info('At time: %s (us) ---- UAV: %s should wait for %s to countdown its back-off counter',
                        self.env.now, self.my_drone.identifier, to_wait)
            start_time = self.env.now  # start to wait

            try:
                yield self.env.timeout(to_wait)
                to_wait = 0  # to break the while loop

                key = ''.join(['mac_send', str(self.my_drone.identifier), '_', str(pkd.packet_id)])

                self.my_drone.mac_process_finish[key] = 1  # mark the process as "finished"

                # occupy the channel to send packet
                with self.channel_states[self.my_drone.identifier].request() as req:
                    yield req

                    logger.info('At time: %s (us) ---- UAV: %s can send packet (pkd id: %s)',
                                self.env.now, self.my_drone.identifier, pkd.packet_id)

                    pkd.transmitting_start_time = self.env.now
                    transmission_mode = pkd.transmission_mode

                    if transmission_mode == 0:  # for unicast
                        next_hop_id = pkd.next_hop_id

                        pkd.increase_ttl()
                        self.phy.unicast(pkd, next_hop_id)  # note: unicast function should be executed first!
                        yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)  # transmission delay

                        # only unicast data packets need to wait for ACK
                        logger.info('At time: %s (us) ---- UAV: %s starts to wait ACK for packet: %s',
                                    self.env.now, self.my_drone.identifier, pkd.packet_id)

                        if self.enable_ack:
                            # used to identify the process of waiting ack
                            key2 = ''.join(['wait_ack', str(self.my_drone.identifier), '_', str(pkd.packet_id)])

                            self.wait_ack_process = self.env.process(self.wait_ack(pkd))
                            self.wait_ack_process_dict[key2] = self.wait_ack_process
                            self.wait_ack_process_finish[key2] = 0  # indicate that this process hasn't finished

                            # continue to occupy the channel to prevent the ACK from being interfered
                            yield self.env.timeout(config.SIFS_DURATION + config.ACK_PACKET_LENGTH / config.BIT_RATE * 1e6)

                    elif transmission_mode == 1:
                        pkd.increase_ttl()
                        self.phy.broadcast(pkd)
                        yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)

            except simpy.Interrupt:
                already_wait = self.env.now - start_time
                logger.info('At time: %s (us) ---- The back-off process of UAV: %s was interrupted, it has been waiting'
                            ' for: %s, original to_wait is: %s',
                            self.env.now, self.my_drone.identifier, already_wait, to_wait)

                to_wait -= already_wait  # the remaining waiting time

                if to_wait > backoff:
                    # interrupted in the process of DIFS
                    to_wait = config.DIFS_DURATION + backoff
                else:
                    # interrupted in the process of backoff, freeze the backoff
                    backoff = to_wait  # remaining backoff time
                    to_wait = config.DIFS_DURATION + backoff

    def wait_ack(self, pkd):
        """
        If ACK is received within the specified time, the transmission is successful, otherwise,
        a re-transmission will be originated
        :param pkd: the data packet that waits for ACK
        :return: none
        """

        try:
            yield self.env.timeout(config.ACK_TIMEOUT)
            self.my_drone.routing_protocol.penalize(pkd)

            logger.info('At time: %s (us) ---- ACK timeout of packet: %s',
                        self.env.now, pkd.packet_id)

            if pkd.number_retransmission_attempt[self.my_drone.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                yield self.env.process(self.my_drone.packet_coming(pkd))
            else:
                self.simulator.metrics.mac_delay.append((self.simulator.env.now - pkd.first_attempt_time) / 1e3)

                key2 = ''.join(['wait_ack', str(self.my_drone.identifier), '_', str(pkd.packet_id)])

                self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1

                logger.info('At time: %s (us) ---- Packet: %s is dropped!',
                            self.env.now, pkd.packet_id)

        except simpy.Interrupt:
            # receive ACK in time
            logger.info('At time: %s (us) ---- UAV: %s receives the ACK for data packet: %s',
                        self.env.now, self.my_drone.identifier, pkd.packet_id)

    def wait_idle_channel(self, channel_id):
        while self.simulator.channel.is_busy_for(self.my_drone, channel_id):
            yield self.env.timeout(config.SLOT_DURATION)

    def listen(self, pkd):
        logger.info('At time: %s (us) ---- UAV: %s starts to listen the channel and perform back-off',
                     self.env.now, self.my_drone.identifier)

        key = ''.join(['mac_send', str(self.my_drone.identifier), '_', str(pkd.packet_id)])

        while self.my_drone.mac_process_finish[key] == 0:  # interrupt only if the process is not complete
            if self.simulator.channel.is_busy_for(self.my_drone, pkd.channel_id):
                # found channel be occupied, start interrupt

                key = ''.join(['mac_send',str(self.my_drone.identifier),'_',str(pkd.packet_id)])
                if not self.my_drone.mac_process_dict[key].triggered:
                    self.my_drone.mac_process_dict[key].interrupt()
                    break
            else:
                pass

            yield self.env.timeout(1)
