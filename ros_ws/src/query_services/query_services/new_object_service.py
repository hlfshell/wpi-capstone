import builtin_interfaces
import rclpy
from capstone_interfaces.msg import ObjectSpotted, StateObject
from capstone_interfaces.srv import AddObject
from query_services import database_functions
from rclpy.node import Node


class SpotNewObjectService(Node):
    def __init__(self):
        super().__init__("spot_new_object_service")
        self.conn = database_functions.create_connection(self, r"state_db.db")
        self.srv = self.create_service(AddObject, "add_new_object", self.add_object)
        database_functions.create_object_table(self.conn, self)

    def add_object(self, request, response):
        self.get_logger().info("Received:\n" + str(request.object_info))
        self.get_logger().info(str(type(request)))

        object_to_add = StateObject()

        object_to_add.description = request.object_info.description
        object_to_add.x = request.object_info.x
        object_to_add.y = request.object_info.y
        object_to_add.z = request.object_info.z
        object_to_add.time_seen = request.object_info.time_seen
        object_to_add.location = self.determine_room(
            request.object_info.x, request.object_info.y, request.object_info.z
        )

        self.get_logger().info(str(object_to_add))

        response.object_state.description = object_to_add.description

        response.object_state.x = object_to_add.x
        response.object_state.y = object_to_add.y
        response.object_state.z = object_to_add.z
        response.object_state.time_seen = object_to_add.time_seen
        response.object_state.location = object_to_add.location

        self.get_logger().info(str(response))

        with self.conn:
            last_row_id = database_functions.create_object(self.conn, object_to_add)

        response.object_state.id = last_row_id

        self.get_logger().info(
            "Object:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Time: %s"
            % (
                response.object_state.id,
                response.object_state.description,
                response.object_state.location,
                response.object_state.x,
                response.object_state.y,
                response.object_state.z,
                str(response.object_state.time_seen),
            )
        )

        self.get_logger().info(
            "response:\n" + str(response) + "\n" + str(type(response))
        )
        return response

    def determine_room(self, x: float, y: float, z: float) -> str:
        # TODO - make it use the map!
        return ""
        # Depends on simulation room, since we are assuming we have a map apriori
        # Todo: work with actual simulated home

        if x > 0 and x <= 10 and y > 0 and y < 10:  # first quadrant (TR)
            return "Living Room"
        elif x > -10 and x <= 0 and y > 0 and y < 10:  # second quadrant (TL)
            return "Bathroom"
        elif x > -10 and x <= 0 and y > -10 and y <= 0:  # third quadrant (BL):
            return "Bedroom"
        elif x > 0 and x <= 10 and y > -10 and y <= 0:  # fourth quadrant (BR)
            return "Kitchen"
        else:
            return "outside the house"


def main():
    rclpy.init()

    service = SpotNewObjectService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
