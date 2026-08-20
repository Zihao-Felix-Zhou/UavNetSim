import numpy as np
from utils import config
import matplotlib.pyplot as plt
from utils.util_function import euclidean_distance_3d


def calculate_velocity(current_pos, target_pos, moving_speed):
    distance = euclidean_distance_3d(current_pos, target_pos)
    normalized_vector = [(target_pos[0] - current_pos[0]) / distance,
                         (target_pos[1] - current_pos[1]) / distance,
                         (target_pos[2] - current_pos[2]) / distance]
    moving_speed = [moving_speed] * 3
    velocity = [d * v for d, v in zip(normalized_vector, moving_speed)]
    return velocity

class PathFollowing3D:
    """
    This path following class will be used when you calculate the flight trajectory in advance (usually in the
    form of "waypoint"), and this class will let the drone fly according to the path you want. It is similar to
    3D Random Waypoint mobility model, the difference is in here, the waypoint is not generated randomly, but is
    set as a parameter "path" instead.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2025/7/9
    Updated at: 2025/7/9
    """

    def __init__(self, drone, path):
        self.my_drone = drone
        self.my_drone.simulator.airspace.validate_path(path)
        self.waypoint_coords = path[1:]  # remove the first waypoint, which is the starting pos
        self.position_update_interval = 1 * 1e5  # 0.1s
        self.pause_time = 0  # no pause

        # determine if the waypoint is reached
        self.arrival_flag = 3  # in meter

        # used to determine if the waypoint has been visited
        self.waypoint_visited = [0 for _ in range(len(path)-1)]

        self.trajectory = []

        self.my_drone.simulator.env.process(self.mobility_update(self.my_drone))
        self.my_drone.simulator.env.process(self.show_trajectory())

    def get_first_unvisited_waypoint(self):
        if 0 in self.waypoint_visited:
            waypoint_idx = self.waypoint_visited.index(0)
            waypoint_coords = self.waypoint_coords[waypoint_idx]
            return waypoint_coords, waypoint_idx
        else:
            return self.waypoint_coords[-1], -1

    def mobility_update(self, drone):
        while True:
            env = drone.simulator.env
            drone_id = drone.identifier
            drone_speed = drone.speed
            cur_position = drone.coords
            target_waypoint, target_waypoint_idx = self.get_first_unvisited_waypoint()
            drone.velocity = calculate_velocity(cur_position, target_waypoint, drone_speed)

            # update the position of next time step
            if config.STATIC_CASE == 0:
                next_position_x = cur_position[0] + drone.velocity[0] * self.position_update_interval / 1e6
                next_position_y = cur_position[1] + drone.velocity[1] * self.position_update_interval / 1e6
                next_position_z = cur_position[2] + drone.velocity[2] * self.position_update_interval / 1e6
            else:
                next_position_x = cur_position[0]
                next_position_y = cur_position[1]
                next_position_z = cur_position[2]

            next_position = [next_position_x, next_position_y, next_position_z]

            drone.move_to(next_position, drone.velocity)
            self.trajectory.append(list(drone.coords))

            # judge if the drone has reach the target waypoint
            if euclidean_distance_3d(drone.coords, target_waypoint) < self.arrival_flag:
                self.waypoint_visited[target_waypoint_idx] = 1
                yield env.timeout(self.pause_time)

            yield env.timeout(self.position_update_interval)
            energy_consumption = (self.position_update_interval / 1e6) * drone.energy_model.power_consumption(drone.speed)
            drone.residual_energy -= energy_consumption

    def show_trajectory(self):
        x = []
        y = []
        z = []
        yield self.my_drone.simulator.env.timeout(config.SIM_TIME-1)

        for i in range(len(self.trajectory)):
            x.append(self.trajectory[i][0])
            y.append(self.trajectory[i][1])
            z.append(self.trajectory[i][2])

        plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_xlim(0, config.MAP_LENGTH)
        ax.set_ylim(0, config.MAP_WIDTH)
        ax.set_zlim(0, config.MAP_HEIGHT)

        # maintain the proportion of the x, y and z axes
        ax.set_box_aspect([config.MAP_LENGTH, config.MAP_WIDTH, config.MAP_HEIGHT])

        waypoint_x = [point[0] for point in self.waypoint_coords]
        waypoint_y = [point[1] for point in self.waypoint_coords]
        waypoint_z = [point[2] for point in self.waypoint_coords]
        ax.scatter(waypoint_x, waypoint_y, waypoint_z, c='r')

        x = np.array(x)
        y = np.array(y)
        z = np.array(z)

        ax.plot(x, y, z)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        plt.show()
