from capstone_interfaces.msg import StateObject, ObjectSpotted
from database_functions import *

import rclpy
import builtin_interfaces
from rclpy.node import Node


class SpotNewObjectClient(Node):

    def __init__(self):
        super().__init__('spot_new_object')


        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()




        conn = create_connection()
        if conn is not None:
            create_object_table(conn)
            # test query?
            self.srv = self.create_service(SpotNewObjectService, 'add_new_object', self.add_object)     

    def add_object(self, description: str, location: str, x: float, y: float, z: float, task: str, timestamp:builtin_interfaces/Time) -> list[StateObject]:
        conn = create_connection()
        with conn:
            spotted_object = (description,location,x,y,z,task,timestamp)
            create_object(conn, spotted_object)
            # test query?


def main():
    rclpy.init()

    new_object_service = SpotNewObjectService()

    rclpy.spin(new_object_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()