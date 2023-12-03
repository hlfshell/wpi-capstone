from query_services.room import Room

from typing import List, Tuple, Optional, Dict

import yaml
import numpy as np

import cv2

from random import randint


class SegmentationMap:
    """
    SegmentationMap is a helper class that loads segmentation maps
    and answers the question of "given a set of coordinates what
    room does it belong to?"
    """

    def __init__(self, map_name: str, rooms: List[Room]):
        self.__map_name = map_name
        self.__rooms: Dict[int, Room] = {}

        for room in rooms:
            self.__rooms[room.id] = room

        self.__offset: Tuple[float, float] = (0, 0)
        self.__resolution: float = 0

        self.__map: np.ndarray = None

        self.__load_map()

    def __load_map(self):
        """
        Loads the map file and creates a list of rooms from it.
        """
        segmentation_map_path = f"{self.__map_name}.segmentation.npy"
        yaml_path = f"{self.__map_name}.yaml"

        self.__map = np.load(segmentation_map_path)
        print("read", np.unique(self.__map, return_counts=True))

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        resolution = data["resolution"]
        origin = (data["origin"][0], data["origin"][1])
        robot_offset = (data["robot_offset"][0], data["robot_offset"][1])

        offset = (origin[0] + robot_offset[0], origin[1] + robot_offset[1])

        self.__offset = offset
        self.__resolution = resolution

    def __meter_coordinates_to_pixel_coordinates(self, location: Tuple[float, float]):
        """
        Given a set of meter coordinates, convert it to pixel coordinates
        for our segmentation map
        """
        height = self.__map.shape[0]
        return (
            int((location[0] - self.__offset[0]) / self.__resolution),
            height - int((location[1] - self.__offset[1]) / self.__resolution),
        )

    def to_img(self, labels=False) -> np.ndarray:
        """
        Returns the segmentation map as a human readable colorized image.
        If labels are enabled the resulting index identifier of the room
        is used.
        """
        img = np.zeros_like(self.__map, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        marker_indexes = np.unique(self.__map)

        # Eliminate -1 and 255 values
        marker_indexes = marker_indexes[marker_indexes > 0]
        marker_indexes = marker_indexes[marker_indexes < 255]

        colors = {}
        for index in marker_indexes:
            colors[index] = (randint(0, 255), randint(0, 255), randint(0, 255))

        for index in marker_indexes:
            img[self.__map == index] = colors[index]

        if labels:
            # In the center of each marker polygon we will place a small
            # representing its index
            for index in marker_indexes:
                # Find the center of the polygon
                y, x = np.where(self.__map == index)
                y = int(np.mean(y))
                x = int(np.mean(x))
                cv2.putText(
                    img,
                    str(index),
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

        return img

    def imshow(self, labels=False):
        """
        Shows the segmentation map as a human readable colorized image.
        """
        img = self.to_img(labels=labels)
        cv2.imshow("Segmentation Map", img)
        cv2.waitKey()

    def get_room(self, location: Tuple[float, float]) -> Optional[Room]:
        """
        Returns the room associated with the given coordinates if any.
        """
        pixel_coords = self.__meter_coordinates_to_pixel_coordinates(location)

        index = self.__map[pixel_coords[0], pixel_coords[1]]

        if index == -1 or index == 255:
            # We now are going to take a look at the immediate surrounding pixels
            # if possible, and count the other indexes around it. Depending on
            # the most common index value, we will try to return that if
            # possible. If it's still -1 or 255 we will still return None
            counts: Dict[int, int] = {}
            delta = 8
            for ydelta in range(-delta, delta):
                for xdelta in range(-delta, delta):
                    y = pixel_coords[1] + ydelta
                    x = pixel_coords[0] + xdelta

                    # Safety checks
                    if y < 0 or y >= self.__map.shape[0]:
                        continue
                    elif x < 0 or x >= self.__map.shape[1]:
                        continue

                    index = self.__map[y, x]
                    if index not in counts:
                        counts[index] = 0
                    counts[index] += 1

            # Get the most common index now
            index = max(counts, key=counts.get)
            # One last check - if we're still commonly -1 or 255, return None
            if index == -1 or index == 255:
                return None

        if index not in self.__rooms:
            return None
        else:
            return self.__rooms[index]
