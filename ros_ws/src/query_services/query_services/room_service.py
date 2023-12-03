import rclpy
from rclpy.node import Node

from capstone_interfaces.srv import GetRooms, RoomByCoordinates
from capstone_interfaces.msg import Room as RoomMsg

from query_services.room import Room
from query_services.database_functions import (
    create_connection,
    create_room_table,
)
from query_services.segmentation import SegmentationMap

from threading import Lock

from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from os import path


maps_dir = path.join(get_package_share_directory("query_services"), "maps")


class RoomService(Node):
    """
    RoomService handles state understanding of rooms and provides
    a set of ROS2 services for querying about them.
    """

    def __init__(self, map_name: str = "house"):
        super().__init__("room_service")

        self.__rooms_lock = Lock()
        self.__rooms: List[Room] = []

        self.__initiate_rooms()

        map_path = path.join(maps_dir, map_name)
        self.__segmentation_map = SegmentationMap(map_path, self.__rooms)

        self.__get_rooms_service = self.create_service(
            GetRooms, "/get_rooms", self.__get_room_callback
        )
        self.__room_by_location_service = self.create_service(
            RoomByCoordinates, "/room_by_location", self.__room_by_location_callback
        )

    def __initiate_rooms(self):
        with self.__rooms_lock:
            db = create_connection(self, "state_db.db")
            create_room_table(db, self)

            # Query all existing rooms
            sql = "SELECT name, x, y FROM rooms"

            rows = db.cursor().execute(sql).fetchall()
            for row in rows:
                self.__rooms.append(Room(row[0], (row[1], row[2])))

            db.close()

    def __get_room_by_coordinates(
        self, location: Tuple[float, float]
    ) -> Optional[Room]:
        """
        Returns the room associated with the given coordinates if any.
        """
        return self.__segmentation_map.get_room(location)

    def __get_room_callback(
        self, msg: GetRooms.Request, response: GetRooms.Response
    ) -> GetRooms.Response:
        """
        Callback for the get_room service. Returns all known rooms as Room objects
        """
        with self.__rooms_lock:
            rooms = [
                RoomMsg(name=room.name, location=room.location) for room in self.__rooms
            ]

            response.rooms = rooms
            return response

    def __room_by_location_callback(
        self, request: RoomByCoordinates.Request, response: RoomByCoordinates.Response
    ) -> RoomByCoordinates.Response:
        """
        Callback for the room_by_location service. Returns the room associated with
        the given coordinates if any.
        """
        room = self.__get_room_by_coordinates((request.x, request.y))
        if room is None:
            response.room = RoomMsg()
        else:
            response.room = RoomMsg(name=room.name, location=room.location)

        return response


def main(args=None):
    rclpy.init(args=args)

    node = RoomService()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
