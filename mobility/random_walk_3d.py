import math
import numpy as np
import random
from utils import config
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class RandomWalk3D:
    """
    3-D Random walk mobility model

    In this model, firstly, the drone will randomly choose a direction of motion, and move along this direction for a
    fixed amount of time ("travel_duration"). In addition to this, it is also possible to specify the drone to move a
    fixed distance in this direction. In this code, we assume that the speed of drone is constant.

    Attributes:
        my_drone: the drone which installs this mobility model
        move_counter
        position_update_interval: in us, determine how often the drone updates its position
        travel_duration: specifies the time for a node to move in a certain direction
        b1, b2, b3: safety boundary of x-, y- and z-axis, respectively
        min_x, max_x: Boundaries on the length of the map
        min_y, max_y: Boundaries on the width of the map
        min_z, max_z: Boundaries on the height of the map
        trajectory: list, used to record the flying trajectories of a certain drone

    References:
        [1] Roy, R.R. (2011). Random Walk Mobility. In: Handbook of Mobile Ad Hoc Networks for Mobility Models.
            Springer, Boston, MA. https://doi.org/10.1007/978-1-4419-6050-4_3
        [2] NS-3 for 2D Random walk mobility model:
            https://www.nsnam.org/docs/release/3.20/doxygen/classns3_1_1_random_walk2d_mobility_model.html


    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/20
    Updated at: 2025/1/7
    """
    def __init__(self, drone):
        self.my_drone = drone
        self.move_counter = 1
        self.position_update_interval = 1 * 1e5  # 0.1s
        self.travel_duration = 4 * 1e6  # the travelling time in the new direction

        self.b1 = 1  # safety boundary of x-axis
        self.b2 = 1  # safety boundary of y-axis
        self.b3 = 1  # safety boundary of z-axis

        self.min_x = 0
        self.max_x = config.MAP_LENGTH

        self.min_y = 0
        self.max_y = config.MAP_WIDTH

        self.min_z = 0
        self.max_z = config.MAP_HEIGHT

        self.my_drone.simulator.env.process(self.mobility_update(self.my_drone))
        self.trajectory = []

        self.rng_mobility = random.Random(self.my_drone.identifier+self.my_drone.simulator.seed + 1)

    def mobility_update(self, drone):
        while True:
            if drone.sleep:
                break
            env = drone.simulator.env
            drone_id = drone.identifier
            cur_position = drone.coords
            cur_velocity = drone.velocity
            cur_direction = drone.direction
            cur_pitch = drone.pitch

            # update the position of next time step
            if config.STATIC_CASE == 0:
                next_position_x = cur_position[0] + cur_velocity[0] * self.position_update_interval / 1e6
                next_position_y = cur_position[1] + cur_velocity[1] * self.position_update_interval / 1e6
                next_position_z = cur_position[2] + cur_velocity[2] * self.position_update_interval / 1e6
            else:
                next_position_x = cur_position[0]
                next_position_y = cur_position[1]
                next_position_z = cur_position[2]

            cur_speed = ((cur_velocity[0] ** 2) + (cur_velocity[1] ** 2) + (cur_velocity[2] ** 2)) ** 0.5

            if env.now % self.travel_duration == 0:  # update velocity and direction
                self.move_counter += 1

                next_direction = self.rng_mobility.uniform(0, 2 * math.pi)

                next_pitch = self.rng_mobility.uniform(-math.pi / 2, math.pi / 2)

                next_velocity_x = cur_speed * math.cos(next_direction) * math.cos(next_pitch)
                next_velocity_y = cur_speed * math.sin(next_direction) * math.cos(next_pitch)
                next_velocity_z = cur_speed * math.sin(next_pitch)

                next_position = [next_position_x, next_position_y, next_position_z]

                if drone_id == 6:
                    self.trajectory.append(next_position)

                next_velocity = [next_velocity_x, next_velocity_y, next_velocity_z]
                next_speed = ((next_velocity_x ** 2) + (next_velocity_y ** 2) + (next_velocity_z ** 2)) ** 0.5
            else:
                next_position = [next_position_x, next_position_y, next_position_z]

                # velocity, direction and pitch should stay the same
                next_direction = cur_direction
                next_pitch = cur_pitch
                next_velocity = cur_velocity
                next_speed = ((cur_velocity[0] ** 2) + (cur_velocity[1] ** 2) + (cur_velocity[2] ** 2)) ** 0.5

            # wall rebound
            next_position, next_velocity, next_direction, next_pitch = self.boundary_test(next_position, next_velocity,
                                                                                          next_direction, next_pitch)

            collision = drone.move_to(next_position, next_velocity)
            next_velocity = drone.velocity
            if collision is not None:
                next_direction = math.atan2(next_velocity[1], next_velocity[0])
                next_pitch = math.atan2(next_velocity[2], math.hypot(next_velocity[0], next_velocity[1]))
            drone.direction = next_direction
            drone.pitch = next_pitch

            yield env.timeout(self.position_update_interval)
            energy_consumption = (self.position_update_interval / 1e6) * drone.energy_model.power_consumption(drone.speed)
            drone.residual_energy = max(0.0, drone.residual_energy - energy_consumption)

    def show_trajectory(self):
        x = []
        y = []
        z = []
        yield self.my_drone.simulator.env.timeout(config.SIM_TIME-1)

        if self.my_drone.identifier == 6:
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
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            plt.show()

    # rebound scheme
    def boundary_test(self, next_position, next_velocity, next_direction, next_pitch):
        if next_position[0] < self.min_x + self.b1 or next_position[0] > self.max_x - self.b1:
            next_velocity[0] = -next_velocity[0]
        if next_position[1] < self.min_y + self.b2 or next_position[1] > self.max_y - self.b2:
            next_velocity[1] = -next_velocity[1]
        if next_position[2] < self.min_z + self.b3 or next_position[2] > self.max_z - self.b3:
            next_velocity[2] = -next_velocity[2]

        next_position[0] = max(self.min_x + self.b1, min(next_position[0], self.max_x - self.b1))
        next_position[1] = max(self.min_y + self.b2, min(next_position[1], self.max_y - self.b2))
        next_position[2] = max(self.min_z + self.b3, min(next_position[2], self.max_z - self.b3))

        return next_position, next_velocity, next_direction, next_pitch
