from utils import config


class Phy:
    def __init__(self, mac):
        self.mac = mac
        self.env = mac.env
        self.my_drone = mac.my_drone

    def _consume_transmit_energy(self, packet):
        duration_seconds = packet.packet_length / config.BIT_RATE
        self.my_drone.residual_energy = max(
            0.0,
            self.my_drone.residual_energy - duration_seconds * config.TRANSMITTING_POWER,
        )

    def unicast(self, packet, next_hop_id):
        self._consume_transmit_energy(packet)
        self.my_drone.simulator.channel.transmit(
            packet,
            self.my_drone.identifier,
            [next_hop_id],
        )

    def broadcast(self, packet):
        self._consume_transmit_energy(packet)
        receiver_ids = [
            drone.identifier
            for drone in self.my_drone.simulator.drones
            if drone.identifier != self.my_drone.identifier
        ]
        self.my_drone.simulator.channel.transmit(
            packet,
            self.my_drone.identifier,
            receiver_ids,
        )

    def multicast(self, packet, dst_id_list):
        self._consume_transmit_energy(packet)
        self.my_drone.simulator.channel.transmit(
            packet,
            self.my_drone.identifier,
            dst_id_list,
        )
