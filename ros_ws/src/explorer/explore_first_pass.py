import cv2
import netpbmfile
import array
import numpy as np
from typing import Union, Tuple, Optional
from queue import PriorityQueue

from nav_msgs.msg import OccupancyGrid, MapMetaData

OCCUPIED_PGM = 254
UNKNOWN_PGM = 205
FREE_PGM = 0
OCCUPIED_RGB = [255, 255, 255]
UNKNOWN_RGB = [127, 127, 127]
FREE_RGB = [0, 0, 0]
OCCUPIED = 1
UNKNOWN = -1
FREE = 0


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
            width=map.shape[1],
            height=map.shape[0],
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
    return np.array(occupancy_grid.data).reshape(
        occupancy_grid.info.height, occupancy_grid.info.width
    )


class Explorer:
    """
    Explorer is a wrapper class that, given an occupancy grid,
    will eventually determine the closest "cluster" of unknown
    spots to navigate to, if any.
    """

    def __init__(
        self,
        map: np.ndarray,
        current_location: Tuple[int, int],
        minimum_distance_meters: float = 1.0,
        pixel_to_meters: float = 0.05,
        minimum_unknown_value: float = -10,
    ):
        self.map = map
        self.current_location = current_location
        self.__queue = Queue()
        self.target_location: Optional[Tuple[int, int]] = None
        self.kernel_size = 5
        self.minimum_distance = minimum_distance_meters
        self.pixel_to_meters = pixel_to_meters
        self.minimum_unknown_value = minimum_unknown_value
        self.__process_map()

    def __process_map(self):
        """
        Given our initial map, we are now going to filter our map
        with kernels to try and detect large clusters of unknown
        spots. This is done to prevent small individual hard-to-reach
        unknown spots from dominating our search.
        """
        processed = np.copy(self.map)

        # We have a map of 0's, -1's, and 1's. To detect the cluster of
        # -1's, we will go through each spot in the map and add in the
        # kernel size's surrounding pixels to the current pixel. Thus
        # lower values will be more unknown, and higher values will be
        # more known. First, however, we will convert our known empty
        # and known occupied spots to 0 for this calculation.
        clustered = processed.copy()
        np.where(clustered == OCCUPIED, 0, clustered)

        # Now we perform an operation on each cell adding the surrounding
        # kernel sized cells to its value.
        kernel = np.ones((self.kernel_size, self.kernel_size)).astype(np.float32)
        filtered = cv2.filter2D(clustered.astype(np.float32), -1, kernel)

        self.__processed_map = filtered

        self.search_map = self.generate_map_image()

    def __search(self):
        """
        Perform an A* search to find the closest suitable cluster of unknown
        locations for the robot to explore. __search performs a singular
        step of the process and must be called repeatedly until a target
        is located or NoTargetLocationFound is raised.
        """

        if len(self.__queue) == 0:
            # If there is nothing in the queue, we have no path to any
            # suitable target, and thus call it quits
            raise NoTargetLocationFound

        current: Tuple[int, int] = self.__queue.pop()

        # Is our current position suitable? To be suitable, it must:
        # 1. Be unknown itself
        # 2. Meet a maximum unknown value threshold from the processed
        #   map
        # 3. Be a minimum distance away from the current location
        if (
            self.map[current] == UNKNOWN
            and self.__processed_map[current] <= self.minimum_unknown_value
            and self.__distance(current, self.current_location)
            >= self.__meters_to_pixels(self.minimum_distance)
        ):
            # We have found a suitable target location
            self.target_location = current
            return self.target_location

        # DEBUG - mark the current spot as explored via setting a color
        self.search_map[current] = [0, 255, 0]

        # If the value wasn't suitable for selection, we can then check its neighbors
        neighbors = self.__get_neighbors(current)
        for neighbor in neighbors:
            # If the neighbor is occupied in the map, we will ignore it
            if self.map[neighbor] == OCCUPIED:
                continue

            # We now calculate the cost of the neighbor. The cost is the
            # distance from the start location to the neighbor.
            cost = self.__distance(neighbor, self.current_location)

            # Now append the neighbor to the queue for possible future
            # exploration
            self.__queue.push(neighbor, cost)

    def __get_neighbors(self, origin: Tuple[int, int]) -> list[Tuple[int, int]]:
        """
        Given a point, return a list of all neighboring points, with protections
        against going out of bounds for the current map
        """
        neighbors = []

        # We will check all 8 neighbors, so we will generate a list of
        # all possible neighbors, and then filter out the ones that are
        # out of bounds
        possible_neighbors = [
            (origin[0] - 1, origin[1] - 1),
            (origin[0] - 1, origin[1]),
            (origin[0] - 1, origin[1] + 1),
            (origin[0], origin[1] - 1),
            (origin[0], origin[1] + 1),
            (origin[0] + 1, origin[1] - 1),
            (origin[0] + 1, origin[1]),
            (origin[0] + 1, origin[1] + 1),
        ]

        for neighbor in possible_neighbors:
            if (
                neighbor[0] < 0
                or neighbor[0] >= self.map.shape[0]
                or neighbor[1] < 0
                or neighbor[1] >= self.map.shape[1]
            ):
                # This neighbor is out of bounds, so we will ignore it
                continue

            neighbors.append(neighbor)

        return neighbors

    def __distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        __distance calculates the euclidean distance between a and b in
        pixels
        """
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1])) ** 2

    def __meters_to_pixels(self, meters: float) -> float:
        """
        __meters_to_pixels converts meters to pixels
        """
        return meters / self.pixel_to_meters

    def __pixels_to_meters(self, pixels: float) -> float:
        """
        __pixels_to_meters converts pixels to meters
        """
        return pixels * self.pixel_to_meters

    def __pixel_coordinates_to_meters(
        self, pixel_coordinates: Tuple[int, int]
    ) -> Tuple[float, float]:
        """
        __pixel_coordinates_to_meters converts pixel coordinates to meters
        """
        return (
            self.__pixels_to_meters(pixel_coordinates[0]),
            self.__pixels_to_meters(pixel_coordinates[1]),
        )

    def __meters_coordinates_to_pixels(
        self, meters_coordinates: Tuple[float, float]
    ) -> Tuple[int, int]:
        """
        __meters_coordinates_to_pixels converts meters coordinates to pixels
        """
        return (
            self.__meters_to_pixels(meters_coordinates[0]),
            self.__meters_to_pixels(meters_coordinates[1]),
        )

    def explore(self) -> Optional[Tuple[int, int]]:
        """ "
        explore will attempt to perform an A* search to find the closest
        suitable cluster of unknown locations for the robot to explore.
        If one is found, the target_location is set on the searcher and
        returned. If one is not found, None is returned.
        """
        if self.target_location is not None:
            return self.target_location
        try:
            self.__queue.push(self.current_location)
            while self.target_location is None:
                self.__search()
                return self.target_location
        except NoTargetLocationFound:
            return None

    def generate_map_image(self) -> np.ndarray:
        """
        Generates a human viewable image of the map with origin and
        possibly target location marked.
        """
        img = np.zeros_like(self.map).astype(np.uint8)

        # Assign colors
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Wherever the map is unknown, set it to 127 in the img
        img = np.where(np.expand_dims(self.map, -1) == 1, [0, 0, 0], img)
        img = np.where(np.expand_dims(self.map, -1) == -1, (127, 127, 127), img)
        img = np.where(np.expand_dims(self.map, -1) == 0, (255, 255, 255), img)
        img = img.astype(np.uint8)

        # Draw a circle at the current_location
        # origin = self.__meters_coordinates_to_pixels(self.current_location)
        # origin = (origin[1], origin[0])
        # cv2.circle(img, origin, 2, (0, 255, 0), -1)

        if self.target_location is not None:
            # Draw a circle at the target_location
            target = self.__meters_coordinates_to_pixels(self.target_location)
            target = (target[1], target[0])
            cv2.circle(img, target, 2, (0, 0, 255), -1)

        return img

    def map_show(self, size: Optional[Union[Tuple[int, int], int]] = None):
        """
        Shows the map in a window for debugging. If a size is
        provided the image will be resized to that size. If that
        size is a tuple, it'll be set directly to that (width,
        height). If it's an int, it'll have the largest side set
        and the other side will be scaled to maintain the aspect.
        """
        img = self.generate_map_image()

        if size is not None:
            if isinstance(size, int):
                # We want to scale the image to the largest side
                # being size, and the other side scaled to maintain
                # the aspect ratio
                height, width, _ = img.shape
                if height > width:
                    # Scale the height to size and scale the width
                    # to maintain the aspect ratio
                    scale = size / height
                    width = int(width * scale)
                    img = cv2.resize(img, (width, size))
                else:
                    # Scale the width to size and scale the height
                    # to maintain the aspect ratio
                    scale = size / width
                    height = int(height * scale)
                    img = cv2.resize(img, (size, height))
            elif isinstance(size, tuple):
                img = cv2.resize(img, size)

        cv2.imshow("Map", img)
        cv2.waitKey()


class NoTargetLocationFound(Exception):
    pass


# Import one of our map files
map = netpbmfile.imread("maps/tbw2.pgm")

og = pgm_to_occupancy_map(map)

origin = (1.3, 3.0)

ex = Explorer(occupancy_grid_to_ndarray(og), origin)

ex.map_show(size=800)

print(ex.explore())

print(ex.search_map.shape, ex.search_map.dtype)
sm = cv2.resize(ex.search_map, (800, 750))
cv2.imshow("Search Map", sm)
cv2.waitKey()
