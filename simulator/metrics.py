from collections import defaultdict

import numpy as np


class Metrics:
    """Collect network, MAC, and physical-layer measurements."""

    def __init__(self, simulator):
        self.simulator = simulator
        self.control_packet_num = 0
        self.datapacket_generated = set()
        self.datapacket_arrived = set()
        self.datapacket_generated_num = 0
        self.delivery_time = []
        self.deliver_time_dict = defaultdict(float)
        self.throughput = []
        self.throughput_dict = defaultdict(float)
        self.hop_cnt = []
        self.hop_cnt_dict = defaultdict(float)
        self.mac_delay = []
        self.collision_num = 0
        self.phy_success_num = 0
        self.phy_failure_num = 0

    def record_generated(self, packet):
        self.datapacket_generated_num += 1
        self.datapacket_generated.add(packet.packet_id)
        self.simulator.event_bus.publish(
            "packet_generated",
            self.simulator.env.now,
            packet_id=packet.packet_id,
            source=packet.src_drone.identifier,
            destination=packet.dst_drone.identifier,
            length_bits=packet.packet_length,
            channel=packet.channel_id,
        )

    def record_phy_result(self, success, collided):
        if success:
            self.phy_success_num += 1
        else:
            self.phy_failure_num += 1
        if collided and not success:
            self.collision_num += 1

    def calculate_metrics(self, received_packet):
        latency = self.simulator.env.now - received_packet.creation_time
        self.deliver_time_dict[received_packet.packet_id] = latency
        self.throughput_dict[received_packet.packet_id] = received_packet.packet_length / (latency / 1e6)
        self.hop_cnt_dict[received_packet.packet_id] = received_packet.get_current_ttl()
        self.datapacket_arrived.add(received_packet.packet_id)
        self.simulator.event_bus.publish(
            "packet_delivered",
            self.simulator.env.now,
            packet_id=received_packet.packet_id,
            source=received_packet.src_drone.identifier,
            destination=received_packet.dst_drone.identifier,
            delay_ms=latency / 1e3,
            hops=received_packet.get_current_ttl(),
        )

    @staticmethod
    def _mean(values):
        return float(np.mean(list(values))) if values else 0.0

    def snapshot(self):
        delivered = len(self.datapacket_arrived)
        total_phy = self.phy_success_num + self.phy_failure_num
        return {
            "generated": self.datapacket_generated_num,
            "delivered": delivered,
            "pdr_percent": delivered / self.datapacket_generated_num * 100
            if self.datapacket_generated_num else 0.0,
            "e2e_delay_ms": self._mean(self.deliver_time_dict.values()) / 1e3,
            "routing_load": self.control_packet_num / delivered if delivered else 0.0,
            "throughput_kbps": self._mean(self.throughput_dict.values()) / 1e3,
            "average_hops": self._mean(self.hop_cnt_dict.values()),
            "average_mac_delay_ms": self._mean(self.mac_delay),
            "phy_success": self.phy_success_num,
            "phy_failure": self.phy_failure_num,
            "phy_success_percent": self.phy_success_num / total_phy * 100 if total_phy else 0.0,
            "collisions": self.collision_num,
        }

    def print_metrics(self):
        values = self.snapshot()
        print("Totally sent:", values["generated"], "data packets")
        print("Packet delivery ratio is:", values["pdr_percent"], "%")
        print("Average end-to-end delay is:", values["e2e_delay_ms"], "ms")
        print("Routing load is:", values["routing_load"])
        print("Average throughput is:", values["throughput_kbps"], "Kbps")
        print("Average hop count is:", values["average_hops"])
        print("Collision num is:", values["collisions"])
        print("Average MAC delay is:", values["average_mac_delay_ms"], "ms")
