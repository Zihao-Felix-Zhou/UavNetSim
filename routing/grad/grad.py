import copy
from simulator.log import logger
from routing.grad.grad_packet import GradMessage
from topology.virtual_force.vf_packet import VfPacket
from routing.grad.grad_cost_table import GradCostTable
from utils import config


class Grad:
    """
    Main procedure of GRAd (Gradient Routing in ad hoc networks) (v1.0)

    Slight changes have been made to the original paper:
    On transmitting data packet:
    1) When a drone wishes to send a data packet to a destination for which the cost to the target is known, it will
       embed the data packet in "GradMessage" with the "msg_type" filed set to "M_DATA", and then broadcast
    2) When a drone wishes to send a data packet to another drone for which there is no entry in the cost table, it
       initiates a request process. Similarly, the originator drone will transmit "GradMessage" whose "msg_type" filed
       is set to "M_REQUEST", specifying the destination in the "target_id", initializing the "remaining_value" field
       to "default_request_cost", "accrued_cost" field to "0", and then broadcast

    On receiving packet:
    Some important things that every receiving drone must do:
    1) When the drone receives a message from its neighbor, it debits the "remaining_value" field by one and increases
       the "accrued_cost" by one
    2) update its cost table according to the incoming message

    Other type-related procedures:
    1) if "msg_type" field is "M_DATA", before reaching the destination, only those drones with a lower cost than the
       "remain_value" in the message can further forward
    2) if "msg_type" field is "M_REQUEST", when I am the "target drone", reply message should be launched. For other
       drones, they will always relay the first copy of the message they receive, unless the "remaining_value" field
       has reached zero
    3) if "msg_type" field is "M_REPLY", when I am the "target drone" of this reply message, it means that now I know
       the routing information of the data packets stored in my "waiting_list". Therefore, all data packets destined for
       the originator of REPLY message in "waiting_list" are taken out and put into the "transmitting_queue". For other
       drones, only those drones with a lower cost than the "remain_value" in the REPLY message can further forward

    References:
        [1] Poor R. Gradient routing in ad hoc networks[J]. 2000.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/20
    Updated at: 2025/4/22
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.cost_table = GradCostTable(self.simulator.env, my_drone)
        self.flag = {}
        self.my_drone.mac_protocol.enable_ack = False

    def next_hop_selection(self, packet):
        self.cost_table.purge()  # update cost table

        dst_drone = packet.dst_drone  # the destination of the data packet
        has_route = self.cost_table.has_entry(dst_drone.identifier)
        enquire = True

        if has_route:
            remaining_value = self.cost_table.get_est_cost(dst_drone.identifier)

            config.GL_ID_GRAD_MESSAGE += 1

            # channel assignment
            channel_id = self.my_drone.channel_assigner.channel_assign()

            data_packet_copy = copy.copy(packet)

            grad_message = GradMessage(src_drone=self.my_drone,
                                       dst_drone=dst_drone,
                                       creation_time=self.simulator.env.now,
                                       id_message=config.GL_ID_GRAD_MESSAGE,
                                       message_length=200 + data_packet_copy.packet_length,
                                       message_type="M_DATA",
                                       accrued_cost=0,
                                       remaining_value=remaining_value,
                                       simulator=self.simulator,
                                       channel_id=channel_id)

            grad_message.attached_data_packet = data_packet_copy
            grad_message.attached_data_packet.increase_ttl()
            grad_message.transmission_mode = 1  # broadcast

            return has_route, grad_message, enquire
        else:
            # there is no entry related to "dst_drone" in the cost table
            config.GL_ID_GRAD_MESSAGE += 1

            # channel assignment
            channel_id = self.my_drone.channel_assigner.channel_assign()

            grad_message = GradMessage(src_drone=self.my_drone,
                                       dst_drone=dst_drone,
                                       creation_time=self.simulator.env.now,
                                       id_message=config.GL_ID_GRAD_MESSAGE,
                                       message_length=200,
                                       message_type="M_REQUEST",
                                       accrued_cost=0,
                                       remaining_value=20,
                                       simulator=self.simulator,
                                       channel_id=channel_id)

            grad_message.transmission_mode = 1  # broadcast
            self.simulator.metrics.control_packet_num += 1

            return has_route, grad_message, enquire

    def packet_reception(self, packet, src_drone_id):
        current_time = self.simulator.env.now

        if isinstance(packet, GradMessage):
            packet_copy = copy.copy(packet)
            msg_type = packet_copy.msg_type  # "M_DATA" or "M_REQUEST" or "M_REPLY"

            originator = packet_copy.originator
            target = packet_copy.target

            packet_copy.remaining_value -= 1  # debits the "remaining_value" field by one
            packet_copy.accrued_cost += 1  # increment "accrued_cost" field by one

            self.cost_table.update_entry(packet_copy, current_time)
            # self.cost_table.print_item()

            if msg_type == "M_REQUEST":
                if self.my_drone.identifier == target.identifier:
                    logger.info('At time: %s (us) ---- UAV: %s receives a REQUEST message from UAV: %s, and REPLY '
                                'message should be launched.',
                                self.simulator.env.now, self.my_drone.identifier, src_drone_id)

                    # response the request
                    config.GL_ID_GRAD_MESSAGE += 1

                    # channel assignment
                    channel_id = self.my_drone.channel_assigner.channel_assign()

                    est_cost = self.cost_table.get_est_cost(originator.identifier)

                    grad_message = GradMessage(src_drone=self.my_drone,
                                               dst_drone=originator,
                                               creation_time=self.simulator.env.now,
                                               id_message=config.GL_ID_GRAD_MESSAGE,
                                               message_length=200,
                                               message_type="M_REPLY",
                                               accrued_cost=0,
                                               remaining_value=est_cost,
                                               simulator=self.simulator,
                                               channel_id=channel_id)

                    grad_message.transmission_mode = 1  # broadcast
                    self.simulator.metrics.control_packet_num += 1

                    self.my_drone.transmitting_queue.put(grad_message)

                else:
                    logger.info('At time: %s (us) ---- UAV: %s receives a REQUEST message from UAV: %s',
                                self.simulator.env.now, self.my_drone.identifier, src_drone_id)

                    if packet_copy.remaining_value > 0:
                        if packet_copy.packet_id not in self.flag.keys():  # it is the first time to receive this message
                            self.flag[packet_copy.packet_id] = 1  # mark as "already broadcast"

                            self.simulator.metrics.control_packet_num += 1
                            self.my_drone.transmitting_queue.put(packet_copy)

            elif msg_type == "M_DATA":
                data_packet = packet_copy.attached_data_packet
                if data_packet.dst_drone.identifier == self.my_drone.identifier:  # reach the destination
                    if data_packet.packet_id not in self.simulator.metrics.datapacket_arrived:
                        self.simulator.metrics.calculate_metrics(data_packet)

                        logger.info('At time: %s (us) ---- Data packet: %s is received by destination UAV: %s',
                                    self.simulator.env.now, data_packet.packet_id, self.my_drone.identifier)
                else:
                    if self.my_drone.transmitting_queue.qsize() < self.my_drone.max_queue_size:
                        if packet_copy.remaining_value > 0:
                            # not all the drones hearing this message have entries related to the destination
                            if self.cost_table.has_entry(data_packet.dst_drone.identifier):
                                est_cost = self.cost_table.get_est_cost(data_packet.dst_drone.identifier)
                                if est_cost <= packet_copy.remaining_value:
                                    logger.info('At time: %s (us) ---- UAV: %s further forward the data packet',
                                                self.simulator.env.now, self.my_drone.identifier)

                                    self.my_drone.transmitting_queue.put(packet_copy)
                            else:
                                pass
                        else:
                            pass
                    else:
                        pass

            elif msg_type == "M_REPLY":
                if self.my_drone.identifier == packet_copy.target.identifier:
                    logger.info('At time: %s (us) ---- UAV: %s receives the REPLY message originates from UAV: %s',
                                self.simulator.env.now, self.my_drone.identifier, packet_copy.originator.identifier)

                    # this indicates that there is a path to dst_drone
                    for item in list(self.my_drone.waiting_list):
                        dst_drone = item.dst_drone  # get the destination of data packet
                        if dst_drone.identifier == packet_copy.originator.identifier:
                            self.my_drone.transmitting_queue.put(item)
                            self.my_drone.waiting_list.remove(item)

                else:
                    if packet_copy.remaining_value > 0:
                        if self.cost_table.has_entry(target.identifier):
                            est_cost = self.cost_table.get_est_cost(target.identifier)
                            if est_cost <= packet_copy.remaining_value:
                                self.simulator.metrics.control_packet_num += 1

                                self.my_drone.transmitting_queue.put(packet_copy)
                        else:
                            pass
                    else:
                        pass

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

        else:
            logger.warning('Unknown message type!')

        yield self.simulator.env.timeout(1)
