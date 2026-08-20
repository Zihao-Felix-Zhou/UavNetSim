import numpy as np
from utils import config


def euclidean_distance_3d(p1, p2):
    """
    Calculate the 3-D Euclidean distance between two nodes
    :param p1: the first point
    :param p2: the second point
    :return: Euclidean distance between p1 and p2
    """

    dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5
    return dist


def euclidean_distance_2d(p1, p2):
    """
    Calculate the 2-D Euclidean distance between two nodes
    :param p1: the first point
    :param p2: the second point
    :return: 2-D Euclidean distance between p1 and p2
    """

    dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    return dist


def grid_map():
    """Grid the map for path planning"""

    grid_shape = (int(config.MAP_LENGTH / config.GRID_RESOLUTION),
                  int(config.MAP_WIDTH / config.GRID_RESOLUTION),
                  int(config.MAP_HEIGHT / config.GRID_RESOLUTION))
    grid = np.zeros(grid_shape, dtype=int)

    return grid


