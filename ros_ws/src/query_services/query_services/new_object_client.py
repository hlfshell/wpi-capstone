from capstone_interfaces.msg import StateObject, ObjectSpotted
from capstone_interfaces.srv import AddObject
from query_services import database_functions

import sys

import rclpy
import time
from numpy import uint32
import builtin_interfaces
from rclpy.node import Node


class SpotNewObjectClient(Node):

    def __init__(self):
        super().__init__('spot_new_object_client')
        self.cli = self.create_client(AddObject, 'add_new_object')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddObject.Request()

    def found_object(self, info: ObjectSpotted):# -> StateObject:
        """
        Add objects from given object information:
        : param description
        :return: all 
        """
        self.req.object_info.description = info.description
        self.req.object_info.x = info.x
        self.req.object_info.y = info.y
        self.req.object_info.z = info.z
        self.req.object_info.time_seen = info.time_seen

        print(type(self.req))

        print("TO REMOVE: SENDING REQUEST")
        print(self.req)
        print ("REQUEST SENT")
        self.future = self.cli.call_async(self.req)
        print("future",self.future.result())
        print(self)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()


def main():
    rclpy.init()

    spot_new_object = SpotNewObjectClient()

    obj_info = ObjectSpotted()
    obj_info.description = str(sys.argv[1])
    obj_info.x = float(sys.argv[2])
    obj_info.y = float(sys.argv[3])
    obj_info.z = float(sys.argv[4])

    time_obj = builtin_interfaces.msg.Time()
    time_val = time.time()
    time_obj.sec = int(time_val)
    time_obj.nanosec = int(uint32((time_val-time_obj.sec)*10e9))
    obj_info.time_seen = time_obj

    print(obj_info)
    print(type(obj_info))

    response = spot_new_object.found_object(obj_info)
 
    spot_new_object.get_logger().info(
        'Object:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Task: %s\n  Time: %s' %
        (str(response.object_state.id), response.object_state.description, response.object_state.location, 
        str(response.object_state.x),str(response.object_state.y),str(response.object_state.z),response.object_state.task_when_seen,str(response.object_state.time_seen)))

    spot_new_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()