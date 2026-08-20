from utils import config


def get_random_start_point_3d(sim_seed, number_of_drones, airspace):
    return airspace.random_positions(sim_seed, number_of_drones, config.UAV_INITIAL_SEPARATION)


def get_customized_start_point_3d(airspace):
    start_positions = []
    for i in range(config.NUMBER_OF_DRONES):
        input_str = input('Please input the coordinates of drone, e.g., 10, 20, 1')
        position_x, position_y, position_z = map(float, input_str.split(','))
        position = (position_x, position_y, position_z)
        if not airspace.position_is_free(position):
            raise ValueError(f'UAV {i} initial position is outside free airspace')
        start_positions.append(position)

    return start_positions
