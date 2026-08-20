import math
import logging
import numpy as np
from topology.virtual_force.vf_packet import VfPacket
from topology.virtual_force.vf_neighbor_table import VfNeighborTable
from utils.util_function import euclidean_distance_3d
import matplotlib.pyplot as plt
from utils import config


# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class VfMotionController:
    """
    Main procedure of motion controller

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the GPSR
        neighbor_table: used to record the neighbor's information
        position_update_interval: in microsecond, determine how often the drone updates its position
        max_step: in meter, the maximum moving distance in each round
        pause_time: in microsecond, time spent waiting for ACK
        next_position: the position to which the drone needs to move in each round

    References:
        [1] Liu. H, et al.,"Simple Movement Control Algorithm for Bi-connectivity in Robotic Sensor Networks,"
            IEEE Journal on Selected Areas in Communications, vol. 28, no. 7, pp. 994-1005, 2010.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/5/20
    Updated at: 2025/3/28
    """

    def __init__(self, drone):
        self.simulator = drone.simulator
        self.my_drone = drone

        self.min_x = 0
        self.max_x = config.MAP_LENGTH

        self.min_y = 0
        self.max_y = config.MAP_WIDTH

        self.min_z = 0
        self.max_z = config.MAP_HEIGHT

        self.neighbor_table = VfNeighborTable(drone.simulator.env, drone)
        self.position_update_interval = 1 * 1e5  # 0.1s
        self.max_step = 20
        self.pause_time = 1 * 1e6
        self.next_position = self.get_next_position()

        self.simulator.env.process(self.initialization())
        self.simulator.env.process(self.motion_control(drone))
        self.trajectory = []
        self.my_drone.simulator.env.process(self.show_trajectory())

    # determine the next target position
    def get_next_position(self):
        self.neighbor_table.purge()  # update the neighbor table

        attractive_force = self.neighbor_table.attractive_force()
        repulsive_force = self.neighbor_table.repulsive_force()

        resultant_force = list(np.array(attractive_force) + np.array(repulsive_force))
        force_magnitude = math.sqrt(sum([item ** 2 for item in resultant_force]))
        force_magnitude_list = [math.sqrt(sum([item ** 2 for item in resultant_force]))] * 3
        force_direction = [a / b for a, b in zip(resultant_force, force_magnitude_list)]

        moving_distance = [math.atan(force_magnitude) * (2 / math.pi) * self.max_step] * 3
        position_shift = [c * d for c, d in zip(moving_distance, force_direction)]

        next_position = list(np.array(self.my_drone.coords) + np.array(position_shift))

        return next_position, force_direction

    def initialization(self):
        config.GL_ID_VF_PACKET += 1

        # channel assignment
        channel_id = self.my_drone.channel_assigner.channel_assign()

        hello_msg = VfPacket(src_drone=self.my_drone,
                             creation_time=self.simulator.env.now,
                             id_hello_packet=config.GL_ID_VF_PACKET,
                             hello_packet_length=config.HELLO_PACKET_LENGTH,
                             simulator=self.simulator,
                             channel_id=channel_id)
        hello_msg.transmission_mode = 1

        logging.info('At time: %s, UAV: %s has motion control hello packet to broadcast, pkd_id is: %s',
                     self.simulator.env.now, self.my_drone.identifier, hello_msg.packet_id)

        yield self.simulator.env.timeout(10)
        self.my_drone.transmitting_queue.put(hello_msg)

    def motion_control(self, drone):
        while True:
            env = drone.simulator.env
            drone_id = drone.identifier
            drone_speed = drone.speed
            cur_position = drone.coords

            # update the position of next time step
            if config.STATIC_CASE == 0:
                self.next_position, force_direction = self.get_next_position()  # update next position

                drone.velocity[0] = drone_speed * force_direction[0]
                drone.velocity[1] = drone_speed * force_direction[1]
                drone.velocity[2] = drone_speed * force_direction[2]

                next_position_x = cur_position[0] + drone.velocity[0] * self.position_update_interval / 1e6
                next_position_y = cur_position[1] + drone.velocity[1] * self.position_update_interval / 1e6
                next_position_z = cur_position[2] + drone.velocity[2] * self.position_update_interval / 1e6
            else:
                next_position_x = cur_position[0]
                next_position_y = cur_position[1]
                next_position_z = cur_position[2]

            if type(next_position_x) is np.ndarray:
                next_position_x = next_position_x[0]
                next_position_y = next_position_y[0]
                next_position_z = next_position_z[0]

            next_pos = [next_position_x, next_position_y, next_position_z]

            if drone_id == 1:
                self.trajectory.append(next_pos)

            # judge if the drone has reach the target waypoint
            if euclidean_distance_3d(next_pos, self.next_position) < 20:
                config.GL_ID_VF_PACKET += 1

                # channel assignment
                channel_id = self.my_drone.channel_assigner.channel_assign()

                hello_msg = VfPacket(src_drone=self.my_drone,
                                     creation_time=self.simulator.env.now,
                                     id_hello_packet=config.GL_ID_VF_PACKET,
                                     hello_packet_length=config.HELLO_PACKET_LENGTH,
                                     simulator=self.simulator,
                                     channel_id=channel_id)

                hello_msg.transmission_mode = 1

                # Note: it should be noted that when a node finish one round of movement, it needs to stop and
                # broadcast its new location information
                self.my_drone.transmitting_queue.put(hello_msg)

                yield env.timeout(self.pause_time)

                self.next_position = self.get_next_position()

            drone.move_to(next_pos, drone.velocity)
            yield env.timeout(self.position_update_interval)

            # User can choose if energy consumption is considered in their project
            # energy_consumption = (self.position_update_interval / 1e6) * drone.energy_model.power_consumption(drone.speed)
            # drone.residual_energy -= energy_consumption

    def show_trajectory(self):
        x = []
        y = []
        z = []
        yield self.my_drone.simulator.env.timeout(config.SIM_TIME-1)
        if self.my_drone.identifier == 1:
            for i in range(len(self.trajectory)):
                x.append(self.trajectory[i][0])
                y.append(self.trajectory[i][1])
                z.append(self.trajectory[i][2])

            plt.figure()
            ax = plt.axes(projection='3d')
            ax.set_xlim(self.min_x, self.max_x)
            ax.set_ylim(self.min_y, self.max_y)
            ax.set_zlim(self.min_z, self.max_z)

            x = np.array(x)
            y = np.array(y)
            z = np.array(z)

            ax.plot(x, y, z)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()
