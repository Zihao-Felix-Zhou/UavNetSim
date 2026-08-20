import copy
import itertools
import math
from dataclasses import dataclass

import simpy

from phy.sionna_rt import OfflineSionnaRtChannelModel, OnlineSionnaRtChannelModel
from utils import config


@dataclass(slots=True)
class Transmission:
    identifier: int
    packet: object
    transmitter_id: int
    receiver_ids: tuple[int, ...]
    channel_id: int
    power_watt: float
    start_time: float
    end_time: float


@dataclass(slots=True)
class Reception:
    packet: object
    transmitter_id: int
    sinr_db: float
    signal_dbm: float


class Channel:
    def __init__(self, env, simulator, channel_trace=None):
        self.env = env
        self.simulator = simulator
        self.inboxes = {}
        self.transmissions = []
        self._identifiers = itertools.count(1)
        if config.CHANNEL_MODE == "offline":
            if channel_trace is None:
                raise ValueError("Offline channel mode requires a precomputed trace")
            self.channel_model = OfflineSionnaRtChannelModel(simulator.event_bus, channel_trace)
        elif config.CHANNEL_MODE == "online":
            self.channel_model = OnlineSionnaRtChannelModel(simulator.event_bus)
        else:
            raise ValueError(f"Unsupported channel mode: {config.CHANNEL_MODE}")

    def create_inbox_for_receiver(self, identifier):
        inbox = simpy.Store(self.env)
        self.inboxes[identifier] = inbox
        return inbox

    def transmit(self, packet, transmitter_id, receiver_ids):
        duration = packet.packet_length / config.BIT_RATE * 1e6
        transmission = Transmission(
            identifier=next(self._identifiers),
            packet=copy.copy(packet),
            transmitter_id=transmitter_id,
            receiver_ids=tuple(receiver_ids),
            channel_id=packet.channel_id,
            power_watt=config.TRANSMITTING_POWER,
            start_time=self.env.now,
            end_time=self.env.now + duration,
        )
        self._prune()
        self.transmissions.append(transmission)
        self.simulator.event_bus.publish(
            "packet_tx_started",
            self.env.now,
            transmission_id=transmission.identifier,
            packet_id=packet.packet_id,
            packet_type=type(packet).__name__,
            source=transmitter_id,
            destinations=list(receiver_ids),
            channel=packet.channel_id,
            duration_us=duration,
        )
        for receiver_id in receiver_ids:
            if receiver_id != transmitter_id:
                self.env.process(self._deliver(transmission, receiver_id))

    def _prune(self):
        retention = 2 * (config.AVERAGE_PAYLOAD_LENGTH + config.PHY_HEADER_LENGTH) / config.BIT_RATE * 1e6
        cutoff = self.env.now - retention
        self.transmissions = [item for item in self.transmissions if item.end_time >= cutoff]

    @staticmethod
    def _overlap(first, second):
        return max(0.0, min(first.end_time, second.end_time) - max(first.start_time, second.start_time))

    @staticmethod
    def _dbm(power_watt):
        if power_watt <= 0:
            return -200.0
        return 10 * math.log10(power_watt * 1000)

    def _channel_overlaps(self, first_channel, second_channel):
        return abs(first_channel - second_channel) < 5

    def _evaluate(self, target, receiver_id):
        drones = self.simulator.drones
        target_duration = target.end_time - target.start_time
        overlapping = []
        for candidate in self.transmissions:
            if candidate.identifier == target.identifier:
                continue
            if not self._channel_overlaps(target.channel_id, candidate.channel_id):
                continue
            overlap = self._overlap(target, candidate)
            if overlap <= 0:
                continue
            overlapping.append((candidate, overlap / target_duration))
        transmitter_ids = [target.transmitter_id]
        transmitter_ids.extend(candidate.transmitter_id for candidate, _ in overlapping)
        gains = self.channel_model.gains(self.env.now, drones, transmitter_ids, [receiver_id])
        desired_gain = gains[(target.transmitter_id, receiver_id)]
        signal_power = target.power_watt * desired_gain
        interference_power = 0.0
        interferers = []
        for candidate, overlap_ratio in overlapping:
            gain = (
                1.0
                if candidate.transmitter_id == receiver_id
                else gains[(candidate.transmitter_id, receiver_id)]
            )
            contribution = candidate.power_watt * gain * overlap_ratio
            interference_power += contribution
            interferers.append({
                "node_id": candidate.transmitter_id,
                "overlap_ratio": overlap_ratio,
                "power_dbm": self._dbm(contribution),
            })
        denominator = config.noise_power_watt() + interference_power
        sinr_db = 10 * math.log10(signal_power / denominator) if signal_power > 0 else -200.0
        return sinr_db, self._dbm(signal_power), interferers

    def _deliver(self, transmission, receiver_id):
        yield self.env.timeout(transmission.end_time - self.env.now)
        sinr_db, signal_dbm, interferers = self._evaluate(transmission, receiver_id)
        success = sinr_db >= config.SINR_THRESHOLD_DB
        self.simulator.metrics.record_phy_result(success, bool(interferers))
        event_type = "packet_rx_succeeded" if success else "packet_rx_failed"
        self.simulator.event_bus.publish(
            event_type,
            self.env.now,
            transmission_id=transmission.identifier,
            packet_id=transmission.packet.packet_id,
            packet_type=type(transmission.packet).__name__,
            source=transmission.transmitter_id,
            destination=receiver_id,
            channel=transmission.channel_id,
            sinr_db=sinr_db,
            signal_dbm=signal_dbm,
            interferers=interferers,
            reason=None if success else "sinr_below_threshold",
        )
        if success:
            reception = Reception(
                packet=copy.copy(transmission.packet),
                transmitter_id=transmission.transmitter_id,
                sinr_db=sinr_db,
                signal_dbm=signal_dbm,
            )
            yield self.inboxes[receiver_id].put(reception)

    def current_transmitters(self, channel_id=None):
        result = []
        for transmission in self.transmissions:
            if transmission.start_time <= self.env.now < transmission.end_time:
                if channel_id is None or self._channel_overlaps(channel_id, transmission.channel_id):
                    result.append(transmission)
        return result

    def is_busy_for(self, drone, channel_id):
        transmissions = [
            transmission for transmission in self.current_transmitters(channel_id)
            if transmission.transmitter_id != drone.identifier
        ]
        if not transmissions:
            return False
        gains = self.channel_model.gains(
            self.env.now,
            self.simulator.drones,
            [transmission.transmitter_id for transmission in transmissions],
            [drone.identifier],
        )
        sensed_power = 0.0
        for transmission in transmissions:
            gain = gains[(transmission.transmitter_id, drone.identifier)]
            sensed_power += transmission.power_watt * gain
        return self._dbm(sensed_power) >= config.CCA_THRESHOLD_DBM

    def point_to_point_sinr(self, receiver_id, transmitter_id, channel_id):
        transmissions = [
            transmission for transmission in self.current_transmitters(channel_id)
            if transmission.transmitter_id != transmitter_id
        ]
        transmitter_ids = [transmitter_id]
        transmitter_ids.extend(transmission.transmitter_id for transmission in transmissions)
        gains = self.channel_model.gains(
            self.env.now, self.simulator.drones, transmitter_ids, [receiver_id]
        )
        gain = gains[(transmitter_id, receiver_id)]
        signal = config.TRANSMITTING_POWER * gain
        interference = 0.0
        for transmission in transmissions:
            other_gain = (
                1.0
                if transmission.transmitter_id == receiver_id
                else gains[(transmission.transmitter_id, receiver_id)]
            )
            interference += transmission.power_watt * other_gain
        if signal <= 0:
            return -200.0
        return 10 * math.log10(signal / (config.noise_power_watt() + interference))

    def close(self):
        self.channel_model.close()
