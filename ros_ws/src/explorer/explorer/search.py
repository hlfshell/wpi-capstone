from typing import Dict, Optional, Tuple, Union

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid

from explorer.constants import FREE, OCCUPIED, UNKNOWN
from explorer.utils import Queue, occupancy_grid_to_ndarray


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
        maximum_obstacle_value: float = 3,
        robot_radius: float = 0.04,
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
        self.maximum_obstacle_value = maximum_obstacle_value
        self.robot_radius = robot_radius
        self.debug = debug
        self.current_location = robot_location
        self.current_location_pixels = self.__meters_coordinates_to_pixels(
            robot_location
        )
        self.__process_maps()
        self.__explored: Dict[Tuple[int, int], bool] = {}

        # __costs will track the costs of a given set of coordinates
        # for expanding costs over distance searched
        self.__costs: Dict[Tuple[int, int], float] = {}

        if self.debug:
            self.__debug_map = self.generate_map_image()

    def __process_maps(self):
        """
        Given our initial map, we are now going to filter our map
        with kernels to try and detect large clusters of unknown
        spots. This is done to prevent small individual hard-to-reach
        unknown spots from dominating our search.

        We then use the same technique to generate a "obstacle"
        proximity map, wherein we rate how close certain cells are to
        obstacles. This is to fight against the explorer honing in on
        cells that are *just* beyond the walls as a target location.
        """
        unknown_map = np.copy(self.map)
        obstacle_map = np.copy(self.map)

        # We have a map of 0's, -1's, and 1's. To detect the cluster of
        # -1's, we will go through each spot in the map and add in the
        # kernel size's surrounding pixels to the current pixel. Thus
        # lower values will be more unknown, and higher values will be
        # more known. First, however, we will convert our known empty
        # and known occupied spots to 0 for this calculation.
        clustered = unknown_map.copy()
        np.where(clustered == OCCUPIED, 0, clustered)

        # Now we perform an operation on each cell adding the surrounding
        # kernel sized cells to its value.
        kernel = np.ones((self.kernel_size, self.kernel_size)).astype(np.float32)
        filtered = cv2.filter2D(clustered.astype(np.float32), -1, kernel)

        self.__unknown_map = filtered

        # Now we will do the same process for the obstacle map; we will
        # be ignoring the unknown squares though
        clustered = obstacle_map.copy()
        np.where(clustered == UNKNOWN, 0, clustered)

        filtered = cv2.filter2D(clustered.astype(np.float32), -1, kernel)

        self.__obstacle_map = filtered

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
        current_meters = self.__pixel_coordinates_to_meters(current)
        if current not in self.__costs:
            current_cost = 0
        else:
            current_cost = self.__costs[current]

        if self.debug:
            self.__debug_map[current] = [255, 0, 0]

        # Is our current position suitable? To be suitable, it must:
        # 1. Be unknown itself
        # 2. Meet a maximum unknown value threshold from the unknown
        #   map
        # 3. Meet a minimum obstacle value threshold from the
        #   obstacle map
        # 4. Be a minimum distance away from the current location
        # 5. Touches at least one other known spot
        current_value = self.map[current]
        unknown_value = self.__unknown_map[current]
        obstacle_value = self.__obstacle_map[current]
        current_distance = self.distance(self.current_location, current_meters)
        neighbors = self.__get_neighbors(current)
        has_known_neighbor = False
        # Determine if any of the neighbors are FREE, If not, we will
        # not consider this location as a potential target
        for neighbor in neighbors:
            if self.map[neighbor] == FREE:
                has_known_neighbor = True
                break
        obstacle_free = self.__check_chonk_radius(current)

        if (
            current_value == UNKNOWN
            and unknown_value <= self.minimum_unknown_value
            and obstacle_value <= self.maximum_obstacle_value
            and current_distance >= self.minimum_distance
            and has_known_neighbor
            and obstacle_free
        ):
            # We have found a suitable target location
            self.target_location_pixels = current
            self.target_location = self.__pixel_coordinates_to_meters(
                self.target_location_pixels
            )
            return self.target_location

        # If the value wasn't suitable for selection, we can then check its neighbors
        for neighbor in neighbors:
            # If we've considered this neighbor before, don't reconsider
            # it as a potential neighbor here.
            if neighbor in self.__explored:
                continue

            # Mark this neighbor as considered
            self.__explored[neighbor] = True

            # If the neighbor is occupied in the map, we will ignore it
            if self.map[neighbor] == OCCUPIED:
                continue

            # Similarly, if the neighbor is close to too many obstacles,
            # we won't consider it.
            if obstacle_value > self.maximum_obstacle_value:
                continue

            # We now calculate the cost of the neighbor. The cost is the
            # distance from the start location to the neighbor, since a
            # greater distance should be a lower score, and lowest score
            # is pulled first.
            neighbor_meters = self.__pixel_coordinates_to_meters(neighbor)
            cost = self.distance(neighbor_meters, current_meters) + current_cost
            self.__costs[neighbor] = cost

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

            return self.target_location
        except NoTargetLocationFound:
            return None
        except Exception as e:
            self.show_debug_map(size=800)
            raise e

    def __check_chonk_radius(self, location: Tuple[int, int]) -> bool:
        """
        __check_chonk_radius checks if the given location has within
        its area a set amount of obstacles, thus preventing the robot
        from traveling through it. For instance - imagine a cylindrical
        pole being viewed from one side of the robot's LIDAR. To the
        robot, the interior of the pole might be an unknown target spot,
        but we know from common sense that the robot can't fit into the
        pole, so don't focus there.

        Returns True if the spot is safe, False otherwise.
        """
        obstacle_map = np.where(self.map == UNKNOWN, 0, map)
        obstacle_map = np.where(self.map == FREE, 0, map)

        # Given the location, find the pixel points within a radial
        # distance of that point.
        radius = self.__meters_to_pixels(self.robot_radius)
        x, y = location

        x_range = range(x - radius, x + radius + 1)
        y_range = range(y - radius, y + radius + 1)
        location_meters = self.__pixel_coordinates_to_meters(location)

        for x in x_range:
            for y in y_range:
                # If the spot is not an obstacle, we don't need to do
                # any further checks
                if obstacle_map[(x, y)] != OCCUPIED:
                    continue

                # Since we don't want a square bounding box but a closer
                # radial one, we need to determine if the given (x,y)
                # point is within the radius of the center point
                xy_meters = self.__pixel_coordinates_to_meters((x, y))
                if self.distance(xy_meters, location_meters) <= radius:
                    # We have encountered an obstacle within our radius
                    # so therefore we can stop - this spot has failed.
                    return False

        return True

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

    def distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        __distance calculates the euclidean distance between a and b
        """
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

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
            coordinates[0] + self.world_origin.position.x,
            coordinates[1] + self.world_origin.position.y,
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
            int(self.__meters_to_pixels(coordinates[0])),
            int(self.__meters_to_pixels(coordinates[1])),
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

    def generate_debug_map_image(
        self, size: Optional[Union[Tuple[int, int], int]] = None
    ) -> np.ndarray:
        """
        Generates a human viewable image of the map with origin and
        possibly target location marked. All considered nodes are
        colored blue if search has been completed. Note that the
        explorer had to have the debug flag set to True for this to
        be populated.
        """
        img = self.__debug_map.copy()

        origin = self.current_location_pixels
        img[origin] = [0, 255, 0]

        if self.target_location is not None:
            target = self.target_location_pixels
            img[target] = [0, 0, 255]

        if size is not None:
            img = self.__resize_map_image(img, size)

        return img

    def show_debug_map(self, size: Optional[Union[Tuple[int, int], int]] = None):
        """
        Shows the map with all considered nodes colored blue. Note
        that the explorer had to have the debug flag set to True
        for this to be populated.
        """
        img = self.generate_debug_map_image(size=size)

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

        # Draw a circle at the current_location - note the drawing
        # is row major thus (y, x)
        origin = self.current_location_pixels
        origin = (origin[1], origin[0])
        cv2.circle(img, origin, 2, (0, 255, 0), -1)

        if self.target_location is not None:
            # Draw a circle at the target_location
            target = self.target_location_pixels
            target = (target[1], target[0])
            cv2.circle(img, target, 2, (0, 0, 255), -1)

        if size is not None:
            img = self.__resize_map_image(img, size)

        cv2.imshow("Map", img)
        cv2.waitKey()


class NoTargetLocationFound(Exception):
    pass
