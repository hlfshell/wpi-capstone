from capstone_interfaces.srv import ObjectIDQuery
from capstone_interfaces.msg import StateObject
import sqlite3
import sys # for testing at command line
from query_services import database_functions

import rclpy
import builtin_interfaces
from rclpy.node import Node


class IDQueryClient(Node):

    def __init__(self):
        super().__init__('id_query_client')
        self.cli = self.create_client(ObjectIDQuery, 'object_id_query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ObjectIDQuery.Request()

    def object_query(self, obj_id: str) -> list[StateObject]:
        """
        Query objects by id
        Takes in id of an object to send service, returns list of StateObjects from service
        """
        self.req.object_id = obj_id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()


def main():
    rclpy.init()

    id_query_client = IDQueryClient()
    response = id_query_client.object_query(int(sys.argv[1]))

    for i in range(len(response.states_of_objects)):
        searched_object = response.states_of_objects[i]
        id_query_client.get_logger().info(
            'ID Searched: %s\nObject:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Time: %s' %
            (str(sys.argv[1]), searched_object.id, searched_object.description, searched_object.location, 
            searched_object.x,searched_object.y,searched_object.z,searched_object.time_seen))

    rclpy.spin(id_query_client)
    id_query_client.destroy_node()


    rclpy.shutdown()


if __name__ == '__main__':
    main()