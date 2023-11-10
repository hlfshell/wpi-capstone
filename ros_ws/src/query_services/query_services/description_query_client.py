from capstone_interfaces.srv import ObjectDescriptionQuery, ObjectIDQuery
from capstone_interfaces.msg import StateObject
import sqlite3
import sys # for testing at command line
# import database_functions

import rclpy
import builtin_interfaces
from rclpy.node import Node


class DescriptionQueryClient(Node):

    def __init__(self):
        super().__init__('description_query_client')
        self.cli = self.create_client(ObjectDescriptionQuery, 'object_description_query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ObjectDescriptionQuery.Request()

    def object_query(self, description: str) -> list[StateObject]:
        """
        Query objects by description
        : param description
        :return: all 
        """
        self.req.object_description = description
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()


def main():
    rclpy.init()

    description_client = DescriptionQueryClient()
    response = description_client.object_query(str(sys.argv[1]))
    for i in len(response):
        """ i.id
        i.description
        i.location
        i.x
        i.y
        i.z
        i.task_when_seen
        i.time_seen """

        description_client.get_logger().info(
            'Description Searched: %s\nObject:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Task: %s\n  Time: %s' %
            (str(sys.argv[1]), response.StateObject[i].id, response.StateObject[i].description, response.StateObject[i].location, 
            response.StateObject[i].x,response.StateObject[i].y,response.StateObject[i].z, response.StateObject[i].task_when_seen,response.StateObject[i].time_seen))

    rclpy.spin(description_client)
    description_client.destroy_node()


    rclpy.shutdown()


if __name__ == '__main__':
    main()