from queue import PriorityQueue
from typing import Tuple

import netpbmfile
import numpy as np
from explorer.constants import (FREE, FREE_PGM, OCCUPIED, OCCUPIED_PGM,
                                UNKNOWN, UNKNOWN_PGM)
from nav_msgs.msg import MapMetaData, OccupancyGrid


class Queue:
    """
    Queue is just a priority queue wrapper to make it slightly
    easier to work with given our use case.
    """

    def __init__(self):
        self.queue = PriorityQueue()

    def push(self, value: Tuple[int, int], cost=0):
        self.queue.put((cost, value))

    def pop(self) -> Tuple[int, int]:
        return self.queue.get()[1]

    def __len__(self):
        return len(self.queue.queue)


def pgm_to_occupancy_map(data: np.ndarray) -> OccupancyGrid:
    # Sometimes pgms are loaded as read only - this
    # extra step gets us out of that

    map = np.zeros_like(data).astype(np.int8)

    # Set the map to 0 to empty, -1 to unknown, and 1 to occupied
    # to match OccupancyGrid rules
    map = np.where(data == FREE_PGM, FREE, map)
    map = np.where(data == OCCUPIED_PGM, OCCUPIED, map)
    map = np.where(data == UNKNOWN_PGM, UNKNOWN, map)

    occupancy_grid = OccupancyGrid(
        info=MapMetaData(
            resolution=0.05,
            width=map.shape[0],
            height=map.shape[1],
        ),
        data=map.flatten(),
    )

    return occupancy_grid


def occupancy_grid_to_ndarray(occupancy_grid: OccupancyGrid) -> np.ndarray:
    """
    occupancy_grid_to_ndarray converts an OccupancyGrid to a numpy array,
    where we convert the flat data to a 2 dimensional array for easier
    processing
    """
    # Reshape the map from a flattened array to a 2d array. Note
    # that the map is row major, so we need to transpose the
    # sizes here to prevent reading the map being read in a
    # corrupted manner for non-square maps
    map = np.array(occupancy_grid.data).reshape(
        occupancy_grid.info.height, occupancy_grid.info.width
    )

    # Sometimes the occupancy grid maps represent not a 1 for
    # occupied but rather a probability from - to 100 for
    # occupied. We want to assume that anything above a set
    # value is occupied
    map = np.where(map >= 1, OCCUPIED, map)

    # Finally, the map is passed row major, so coordinates are (y,x);
    # I'd much rather reason about the map as (x,y) so we'll transpose
    # the map
    map = map.T

    return map


def load_pgm_file(path: str) -> np.ndarray:
    """
    load_pgm_file loads a pgm file from the given path and returns
    it as a numpy array
    """
    return netpbmfile.imread(path)
