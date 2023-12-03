from __future__ import annotations

from threading import Lock
from typing import List, Optional, Tuple, Union

import cv2
import netpbmfile
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid

from litterbug.brensenham import define_line

OCCUPIED = 1
UNKNOWN = -1
FREE = 0

OCCUPIED_PGM = 0
UNKNOWN_PGM = 205
FREE_PGM = 254

OCCUPIED_RGB = [0, 0, 0]
UNKNOWN_RGB = [127, 127, 127]
FREE_RGB = [255, 255, 255]


class Map:
    """
    Map is a container class that manages a map for detecting
    obstacles and determining simple line-of-sight

    map - the numpy two dimensional array of the map

    resolution - the resolution of the map in meters per pixel

    detection_thickness - when we do a line of sight detection,
        how thick should the lines be when drawing on the map
        to give a margin of error for detection.

    robot_vision_radius - the radius of the robot's vision cone
        in meters

    robot_vision_angle - the angle of the robot's vision cone
        in degrees
    """

    def __init__(
        self,
        map: np.ndarray,
        resolution: float = 0.05,
        origin: Tuple[float, float] = (0, 0),
        detection_thickness: int = 3,
        robot_vision_radius: float = 8,
        robot_vision_angle: float = 90.0,
    ):
        # super().__init__("litterbug_map")
        self.map = map
        self.map_lock = Lock()
        self.__resolution = resolution
        self.origin = origin
        self.__detection_thickness = detection_thickness
        self.__vision_radius = robot_vision_radius
        self.__vision_angle = robot_vision_angle

    @staticmethod
    def FromMapFile(path: str) -> Map:
        """
        FromPGM loads a map from a pgm file and returns
        it as a numpy array, and reads the YAML file
        accompanying the map to get the resolution and
        origin offsets. Note that it assumes we have
        the robot_offset in the yaml file as well, which
        is the offset of the robot from (0,0) when it
        performed the initial mapping.
        """
        map_path = f"{path}.pgm"
        yaml_path = f"{path}.yaml"

        map = netpbmfile.imread(map_path)

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        resolution = data["resolution"]
        origin = (data["origin"][0], data["origin"][1])
        robot_offset = (data["robot_offset"][0], data["robot_offset"][1])

        origin = (
            origin[0] + robot_offset[0],
            origin[1] + robot_offset[1],
        )

        # Set the map to 0 to empty, -1 to unknown, and 1 to occupied
        # to match OccupancyGrid rules
        map = np.where(map == OCCUPIED_PGM, OCCUPIED, map)
        map = np.where(map == FREE_PGM, FREE, map)
        map = np.where(map == UNKNOWN_PGM, UNKNOWN, map)

        map = Map(map, resolution=resolution, origin=origin)

        return map

    def to_rgb(self, size: Optional[Union[Tuple[int, int], int]] = None) -> np.ndarray:
        """
        to_rgb generates an RGB map for visualization. The optional
        size parameter allows you to specify the size of the map
        in pixels either in (w, h), or as a single integer to make
        the largest side of the map that size.
        """
        # rgb_map = np.zeros((self.__map.shape[0], self.__map.shape[1], 3)).astype(
        #     np.uint8
        # )
        rgb_map = np.zeros_like(self.map).astype(np.uint8)
        rgb_map = cv2.cvtColor(rgb_map, cv2.COLOR_GRAY2RGB)

        rgb_map = np.where(
            np.expand_dims(self.map, -1) == OCCUPIED, OCCUPIED_RGB, rgb_map
        )
        rgb_map = np.where(
            np.expand_dims(self.map, -1) == UNKNOWN, UNKNOWN_RGB, rgb_map
        )
        rgb_map = np.where(np.expand_dims(self.map, -1) == FREE, FREE_RGB, rgb_map)

        rgb_map = rgb_map.astype(np.uint8)

        if size is not None:
            rgb_map = self.resize_img(rgb_map, size)
        return rgb_map

    def draw(self, size: Optional[Union[Tuple[int, int], int]] = None):
        """
        draw takes in a map and draws it on screen for debugging
        """
        rgb_map = self.to_rgb(size=size)
        cv2.imshow("Map", cv2.cvtColor(rgb_map, cv2.COLOR_RGB2BGR))
        cv2.waitKey()

    def resize_img(
        self, img: np.ndarray, size: Union[Tuple[int, int], int]
    ) -> np.ndarray:
        """
        __resize img resizes an image to the given size. You can
        specify the size of the map in pixels either in (w, h),
        or as a single integer to make the largest side of the map
        that size.
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

    def __meter_coordinates_to_pixel_coordinates(
        self, point: Tuple[float, float]
    ) -> Tuple[int, int]:
        """
        __meter_coordinates_to_pixel_coordinates converts from
        meter coordinates to pixel coordinates
        """
        x, y = point
        return (
            int((x - self.origin[0]) / self.__resolution),
            int((y - self.origin[1]) / self.__resolution),
        )

    def __pixel_coordinates_to_meter_coordinates(
        self, point: Tuple[int, int]
    ) -> Tuple[float, float]:
        """
        __pixel_coordinates_to_meter_coordinates converts from
        pixel coordinates to meter coordinates
        """
        x, y = point
        return (
            (x * self.__resolution) + self.origin[0],
            (y * self.__resolution) + self.origin[1],
        )

    def __distance(
        self, origin: Tuple[float, float], target: Tuple[float, float]
    ) -> float:
        """
        __distance returns the distance between two points in whatever units
        the coordinates are in
        """
        return np.sqrt((origin[0] - target[0]) ** 2 + (origin[1] - target[1]) ** 2)

    def line(
        self, origin: Tuple[int, int], target: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        line returns a list of cells that the line passes through
        two pixel coordinate locations using Bresenham's algorithm
        modified to use the detection thickness
        """

        return define_line(origin[0], origin[1], target[0], target[1])

    def line_of_sight(
        self, origin: Tuple[float, float], target: Tuple[float, float]
    ) -> bool:
        """
        line_of_sight returns true if there is a clear line of sight
        between the origin and target, where a line drawn between
        the two points via Brensenham's algorithm does not intersect
        with any occupied cells. We modify the algorithm to handle
        a variable line thickness per settings of the map.
        """
        origin = (
            int((origin[0] - self.origin[0]) / self.__resolution),
            self.map.shape[0] - int((origin[1] - self.origin[1]) / self.__resolution),
        )

        target = (
            int((target[0] - self.origin[0]) / self.__resolution),
            self.map.shape[0] - int((target[1] - self.origin[1]) / self.__resolution),
        )

        # Get the cells that the line passes through
        cells = self.line((origin[1], origin[0]), (target[1], target[0]))

        # Now determine if any of the cells in our map are
        # mark OCCUPIED; if so, return False
        for cell in cells:
            if self.map[cell] == OCCUPIED:
                return False

        return True


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


# OCCUPIED_RGB = [255, 255, 255]
# UNKNOWN_RGB = [127, 127, 127]
# FREE_RGB = [0, 0, 0]
