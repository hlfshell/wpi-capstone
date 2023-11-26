import numpy as np
import netpbmfile
import cv2
from typing import Tuple, List

OCCUPIED = 1
UNKNOWN = -1
FREE = 0

OCCUPIED_PGM = 0
UNKNOWN_PGM = 205
FREE_PGM = 254

OCCUPIED_RGB = [255, 255, 255]
UNKNOWN_RGB = [127, 127, 127]
FREE_RGB = [0, 0, 0]


class Map:
    """
    Map is a container class that manages a map for detecting
    obstacles and determining simple line-of-sight

    map - the numpy two dimensional array of the map

    resolution - the resolution of the map in meters per pixel

    detection_thickness - when we do a line of sight detection,
        how thick should the lines be when drawing on the map
        to give a margin of error for detection.
    """

    def __init__(
        self, map: np.ndarray, resolution: float = 0.01, detection_thickness: int = 3
    ):
        self.__map = map
        self.__detection_thickness = detection_thickness

    @staticmethod
    def FromPGM(path: str) -> "Map":
        """
        FromPGM loads a map from a pgm file and returns
        it as a numpy array
        """
        original_map = netpbmfile.imread(path)

        # Get the resolution of the map from the pgm file
        resolution = 1 / float(original_map.meta["resolution"])
        print("resolution", resolution)

        # Sometimes pgms are loaded as read only - this
        # extra step gets us out of that
        map = np.zeros_like(original_map).astype(np.int8)

        # Set the map to 0 to empty, -1 to unknown, and 1 to occupied
        # to match OccupancyGrid rules
        map = np.where(map == FREE_PGM, FREE, original_map)
        map = np.where(map == OCCUPIED_PGM, OCCUPIED, original_map)
        map = np.where(map == UNKNOWN_PGM, UNKNOWN, original_map)

        # The map is stored in row major (y, x) order, and we wish
        # to store it in column major (x, y) order, so we transpose
        # the map
        map = map.T

        return Map(map, resolution=resolution)

    def to_rgb(self) -> np.ndarray:
        """
        to_rgb generates an RGB map for visualization
        """
        rgb_map = np.zeros((self.__map.shape[0], self.__map.shape[1], 3)).astype(
            np.uint8
        )
        rgb_map = np.where(self.__map == OCCUPIED, OCCUPIED_RGB, rgb_map)
        rgb_map = np.where(self.__map == UNKNOWN, UNKNOWN_RGB, rgb_map)
        rgb_map = np.where(self.__map == FREE, FREE_RGB, rgb_map)
        return rgb_map

    def draw(self):
        """
        draw takes in a map and draws it on screen for debugging
        """
        rgb_map = self.to_rgb()
        cv2.imshow("Map", rgb_map)
        cv2.waitKey()

    def __meter_coordinates_to_pixel_coordinates(
        self, x: float, y: float
    ) -> Tuple[int, int]:
        """
        __meter_coordinates_to_pixel_coordinates converts from
        meter coordinates to pixel coordinates
        """
        return (
            int(x / self.__resolution),
            int(y / self.__resolution),
        )

    def line(
        self, origin: Tuple[int, int], target: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        line returns a list of cells that the line passes through
        two pixel coordinate locations using Bresenham's algorithm
        modified to use the detection thickness
        """
        # Initiate our position
        ox, oy = origin
        tx, ty = target

        dx = abs(tx - ox)
        dy = abs(ty - oy)

        # Detect horizontal or vertical lines
        if dx == 0:
            pass
        elif dy == 0:
            pass

        # Now handle the general case
        x_slope = 1 if tx > 0 else -1
        y_slope = 1 if tx > ox else -1
        error = dx - dy

        cells: List[Tuple[int, int]] = []

        while True:
            # Draw each cell within the spot and selected thickness,
            # but note if the cel is out of bounds
            for i in range(-self.__detection_thickness, self.__detection_thickness):
                if (ox + i < 0) or (ox + i >= self.__map.shape[0]):
                    continue
                if (oy + i < 0) or (oy + i >= self.__map.shape[1]):
                    continue

                cells.append(
                    (
                        ox + i,
                        oy,
                    )
                )

            # If we're at the target point, we're done
            if (ox == tx) and (oy == ty):
                break

            # Adjust the line to the next point
            adjustment = 2 * error
            if adjustment > -dy:
                error -= dy
                ox += x_slope
            if adjustment < dx:
                error += dx
                oy += y_slope

        # Return the cells
        return cells

    def line_of_sight(self, origin: Tuple[float, float], target: Tuple[float, float]):
        """
        line_of_sight returns true if there is a clear line of sight
        between the origin and target, where a line drawn between
        the two points via Brensenham's algorithm does not intersect
        with any occupied cells. We modify the algorithm to handle
        a variable line thickness per settings of the map.
        """
        # Convert origin and target to pixel coordinates
        origin = self.__meter_coordinates_to_pixel_coordinates(origin)
        target = self.__meter_coordinates_to_pixel_coordinates(target)

        # Get the cells that the line passes through
        cells = self.line(origin, target)

        # Now determine if any of the cells in our map are
        # mark OCCUPIED; if so, return False
        for cell in cells:
            if self.__map[cell] == OCCUPIED:
                return False

        return True
