import numpy as np
import netpbmfile
import cv2
from typing import Tuple, List, Optional, Union
import yaml

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
        resolution: float = 0.01,
        detection_thickness: int = 3,
        robot_vision_radius: float = 8,
        robot_vision_angle: float = 90.0,
    ):
        self.__map = map
        self.__detection_thickness = detection_thickness
        self.__resolution = resolution
        self.__vision_radius = robot_vision_radius
        self.__vision_angle = robot_vision_angle

    @staticmethod
    def FromMapFile(path: str) -> "Map":
        """
        FromPGM loads a map from a pgm file and returns
        it as a numpy array, and reads the YAML file
        accompanying the map to get the resolution
        """
        map_path = f"{path}.pgm"
        yaml_path = f"{path}.yaml"

        map = netpbmfile.imread(map_path)

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        resolution = data["resolution"]
        print("resolution", resolution)

        # Sometimes pgms are loaded as read only - this
        # extra step gets us out of that
        # map = np.zeros_like(map).astype(np.int8)

        # Set the map to 0 to empty, -1 to unknown, and 1 to occupied
        # to match OccupancyGrid rules
        print("uniques", np.unique(map, return_counts=True))
        map = np.where(map == OCCUPIED_PGM, OCCUPIED, map)
        print("OCCUPIED", np.unique(map, return_counts=True))
        map = np.where(map == FREE_PGM, FREE, map)
        print("FREE", np.unique(map, return_counts=True))
        map = np.where(map == UNKNOWN_PGM, UNKNOWN, map)
        print("uniques", np.unique(map, return_counts=True))
        # The map is stored in row major (y, x) order, and we wish
        # to store it in column major (x, y) order, so we transpose
        # the map
        map = map.T

        return Map(map, resolution=resolution)

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
        rgb_map = np.zeros_like(self.__map).astype(np.uint8)
        rgb_map = cv2.cvtColor(rgb_map, cv2.COLOR_GRAY2RGB)

        rgb_map = np.where(
            np.expand_dims(self.__map, -1) == OCCUPIED, OCCUPIED_RGB, rgb_map
        )
        rgb_map = np.where(
            np.expand_dims(self.__map, -1) == UNKNOWN, UNKNOWN_RGB, rgb_map
        )
        rgb_map = np.where(np.expand_dims(self.__map, -1) == FREE, FREE_RGB, rgb_map)

        rgb_map = rgb_map.astype(np.uint8)

        if size is not None:
            rgb_map = self.__resize_img(rgb_map, size)
        return rgb_map

    def draw(self, size: Optional[Union[Tuple[int, int], int]] = None):
        """
        draw takes in a map and draws it on screen for debugging
        """
        rgb_map = self.to_rgb(size=size)
        print("rgb", rgb_map.shape, rgb_map.dtype)
        cv2.imshow("Map", cv2.cvtColor(rgb_map, cv2.COLOR_RGB2BGR))
        cv2.waitKey()

    def __resize_img(
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
        # Initiate our position
        ox, oy = origin
        tx, ty = target

        dx = abs(tx - ox)
        dy = abs(ty - oy)

        # Detect horizontal or vertical lines, as it breaks the
        # general case below due to division by 0
        if dx == 0:
            # The line is vertical, so draw the set thickness
            # along the x axis only as we move along the y axis
            cells: List[Tuple[int, int]] = []
            start = min(oy, ty)
            end = max(oy, ty)
            for y in range(start, end + 1):
                for dx in range(
                    -self.__detection_thickness, self.__detection_thickness + 1
                ):
                    # Handle out of bounds
                    if (ox + dx < 0) or (ox + dx >= self.__map.shape[0]):
                        continue
                    if (y < 0) or (y >= self.__map.shape[1]):
                        continue

                    cells.append((ox + dx, y))

            return cells
        elif dy == 0:
            # The line is horizontal, so draw the set thickness
            # along the y axis only as we move along the x axis
            cells: List[Tuple[int, int]] = []
            start = min(ox, tx)
            end = max(ox, tx)
            for x in range(start, end + 1):
                for dy in range(
                    -self.__detection_thickness, self.__detection_thickness + 1
                ):
                    # Handle out of bounds
                    if (x < 0) or (x >= self.__map.shape[0]):
                        continue
                    if (oy + dy < 0) or (oy + dy >= self.__map.shape[1]):
                        continue

                    cells.append((x, oy + dy))

            return cells

        # Now handle the general case
        x_slope = 1 if tx > 0 else -1
        y_slope = 1 if tx > ox else -1
        error = dx - dy

        cells: List[Tuple[int, int]] = []

        while True:
            # Draw each cell within the spot and selected thickness,
            # but note if the cel is out of bounds
            for i in range(-self.__detection_thickness, self.__detection_thickness + 1):
                if (ox + i < 0) or (ox + i >= self.__map.shape[0]):
                    continue
                if (oy + i < 0) or (oy + i >= self.__map.shape[1]):
                    continue

                cells.append((ox + i, oy))

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
