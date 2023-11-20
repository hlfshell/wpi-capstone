import cv2
import netpbmfile
import array
import numpy as np
from typing import Union, Tuple, Optional
from queue import PriorityQueue

from nav_msgs.msg import OccupancyGrid, MapMetaData

OCCUPIED = 1
UNKNOWN = -1
FREE = 0

OCCUPIED_PGM = 0
UNKNOWN_PGM = 205
FREE_PGM = 254

OCCUPIED_RGB = [255, 255, 255]
UNKNOWN_RGB = [127, 127, 127]
FREE_RGB = [0, 0, 0]


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
        occupancy_grid: OccupancyGrid,
        robot_location: Tuple[float, float],
        minimum_distance_meters: float = 1.0,
        minimum_unknown_value: float = -10,
        debug: bool = False,
    ):
        self.map = occupancy_grid_to_ndarray(occupancy_grid)

        self.__queue = Queue()
        self.target_location: Optional[Tuple[int, int]] = None
        self.kernel_size = 5
        self.minimum_distance = minimum_distance_meters
        self.pixel_to_meters = occupancy_grid.info.resolution
        self.world_origin = occupancy_grid.info.origin
        self.minimum_unknown_value = minimum_unknown_value
        self.debug = debug

        self.current_location = robot_location
        self.current_location_pixels = self.__meters_coordinates_to_pixels(
            robot_location
        )
        print("starting locations")
        print(self.current_location)
        print(self.current_location_pixels)

        self.__process_map()

        if self.debug:
            self.__debug_map = self.generate_map_image()

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

        if self.debug:
            # self.__debug_map[current[1], current[0]] = [255, 0, 0]
            self.__debug_map[current] = [255, 0, 0]

            print(
                current, self.__distance(current, self.current_location), "meters away"
            )

        # Is our current position suitable? To be suitable, it must:
        # 1. Be unknown itself
        # 2. Meet a maximum unknown value threshold from the processed
        #   map
        # 3. Be a minimum distance away from the current location
        current_value = self.map[current]
        processed_value = self.__processed_map[current]
        current_distance = self.__pixel_coordinates_to_meters(current)
        print(">>", current, current_value, processed_value, current_distance)
        if (
            current_value == UNKNOWN
            and processed_value <= self.minimum_unknown_value
            and current_distance >= self.minimum_distance
        ):
            print("found!", current)
            # We have found a suitable target location
            self.target_location = current
            return self.target_location

        # If the value wasn't suitable for selection, we can then check its neighbors
        neighbors = self.__get_neighbors(current)
        print("neighbors", neighbors)
        for neighbor in neighbors:
            # If the neighbor is occupied in the map, we will ignore it
            if self.map[neighbor] == OCCUPIED:
                continue

            # We now calculate the cost of the neighbor. The cost is the
            # distance from the start location to the neighbor.
            neighbor_meters = self.__pixel_coordinates_to_meters(neighbor)
            cost = self.__distance(neighbor_meters, self.current_location)

            # Now append the neighbor to the queue for possible future
            # exploration
            self.__queue.push(neighbor, cost)

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
            self.__queue.push(self.current_location_pixels, 0)
            while self.target_location is None:
                self.__search()

            # If we've reached this, we have the target location
            self.target_location_pixels = self.__pixel_coordinates_to_meters(
                self.target_location
            )

            return self.target_location
        except NoTargetLocationFound:
            return None
        except Exception as e:
            self.show_debug_map(size=800)
            raise e

    def __get_neighbors(self, origin: Tuple[int, int]) -> list[Tuple[int, int]]:
        """
        Given a point, return a list of all neighboring points, with protections
        against going out of bounds for the current map
        """
        neighbors = []

        # We will check all 8 neighbors, so we will generate a list of
        # all possible neighbors, and then filter out the ones that are
        # out of bounds
        x, y = origin
        neighbors = []
        for xdelta in [-1, 0, 1]:
            for ydelta in [-1, 0, 1]:
                if xdelta == 0 and ydelta == 0:
                    continue
                elif x + xdelta < 0 or x + xdelta >= self.map.shape[0]:
                    continue
                elif y + ydelta < 0 or y + ydelta >= self.map.shape[1]:
                    continue
                else:
                    neighbors.append((x + xdelta, y + ydelta))

        return neighbors

    def __distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        __distance calculates the euclidean distance between a and b in
        pixels
        """
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1])) ** 2

    def __meters_to_pixels(self, meters: float) -> int:
        """
        __meters_to_pixels converts meters to pixels
        """
        return int(meters / self.pixel_to_meters)

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
        coordinates = (
            self.__pixels_to_meters(pixel_coordinates[0]),
            self.__pixels_to_meters(pixel_coordinates[1]),
        )

        # Then offset by the origin
        coordinates = (
            int(coordinates[0] + self.world_origin.position.x),
            int(coordinates[1] + self.world_origin.position.y),
        )

        return coordinates

    def __meters_coordinates_to_pixels(
        self, meters_coordinates: Tuple[float, float]
    ) -> Tuple[int, int]:
        """
        __meters_coordinates_to_pixels converts meters coordinates to pixels
        while noting the origin offset from the occupancy grid; specifically,
        the origin maps to (0,0) but is at some real world location (x,y)
        """
        # First we need to offset the coordinates by the origin)
        coordinates = (
            meters_coordinates[0] - self.world_origin.position.x,
            meters_coordinates[1] - self.world_origin.position.y,
        )

        # then convert to pixels
        coordinates = (
            self.__meters_to_pixels(coordinates[0]),
            self.__meters_to_pixels(coordinates[1]),
        )

        return coordinates

    def __resize_map_image(
        self, img: np.ndarray, size: Optional[Union[Tuple[int, int], int]] = None
    ) -> np.ndarray:
        """
        __resize_map_image resizes the map image to the given size. If
        no size is provided, the image is not resized.
        """
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

        return img

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

        return img

    def show_debug_map(self, size: Optional[Union[Tuple[int, int], int]] = None):
        """ """
        img = self.__debug_map.copy()

        origin = self.current_location_pixels
        # origin = (origin[1], origin[0])
        img[origin] = [0, 255, 0]

        if self.target_location is not None:
            target = self.target_location_pixels
            # target = (target[1], target[0])
            img[target] = [0, 0, 255]

        if size is not None:
            img = self.__resize_map_image(img, size)

        cv2.imshow("Debug Map", img)
        cv2.waitKey()

    def show_map(self, size: Optional[Union[Tuple[int, int], int]] = None):
        """
        Shows the map in a window for debugging. If a size is
        provided the image will be resized to that size. If that
        size is a tuple, it'll be set directly to that (width,
        height). If it's an int, it'll have the largest side set
        and the other side will be scaled to maintain the aspect.
        """
        img = self.generate_map_image()

        # Draw a circle at the current_location
        origin = self.current_location_pixels
        # origin = (origin[1], origin[0])
        cv2.circle(img, origin, 2, (0, 255, 0), -1)

        if self.target_location is not None:
            # Draw a circle at the target_location
            target = self.__meters_coordinates_to_pixels(self.target_location)
            # target = (target[1], target[0])
            cv2.circle(img, target, 2, (0, 0, 255), -1)

        if size is not None:
            img = self.__resize_map_image(img, size)

        cv2.imshow("Map", img)
        cv2.waitKey()


class NoTargetLocationFound(Exception):
    pass


# Import one of our map files
map = netpbmfile.imread("maps/tbw2.pgm")

og = pgm_to_occupancy_map(map)
# map2 = occupancy_grid_to_ndarray(og)

# print(np.unique(map2, return_counts=True))
# cv2.imshow("Map", map2)
# cv2.waitKey()
# img = np.zeros_like(map2).astype(np.uint8)

# # Assign colors
# img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# # Wherever the map is unknown, set it to 127 in the img
# img = np.where(np.expand_dims(map2, -1) == 1, [0, 0, 0], img)
# img = np.where(np.expand_dims(map2, -1) == -1, (127, 127, 127), img)
# img = np.where(np.expand_dims(map2, -1) == 0, (255, 255, 255), img)
# img = img.astype(np.uint8)

# cv2.imshow("Map", img)
# cv2.waitKey()

origin = (1.3, 3.0)

# og.info.origin.position.x = origin[0]
# og.info.origin.position.y = origin[1]
og.info.resolution = 0.05

ex = Explorer(og, origin, debug=True)

print(ex.current_location_pixels)
# print("init", map2[ex.current_location_pixels])

# ex.show_map(size=800)

print(ex.explore())

# ex.show_map(size=800)
ex.show_debug_map(size=800)
