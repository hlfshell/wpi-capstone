from capstone_interfaces.msg import StateObject, ObjectSpotted
from capstone_interfaces.srv import AddObject
from query_services import database_functions

import rclpy
import builtin_interfaces
from rclpy.node import Node


class SpotNewObjectService(Node):

    def __init__(self):
        super().__init__('spot_new_object_service')
        self.conn = database_functions.create_connection(r"state_db.db")
        self.srv = self.create_service(AddObject, 'add_new_object', self.add_object)
        if self.conn is not None:
            database_functions.create_object_table(self.conn) # checks if already made

        #def add_object(self, incoming_object_info: ObjectSpotted, response:StateObject)->StateObject:
    def add_object(self, request, response):
     
        print("Received:\n",request.object_info)
        print(type(request))

        object_to_add = StateObject()

        object_to_add.description = request.object_info.description
        object_to_add.x = request.object_info.x
        object_to_add.y = request.object_info.y
        object_to_add.z = request.object_info.z
        object_to_add.time_seen = request.object_info.time_seen
        object_to_add.location = self.determine_room(request.object_info.x,request.object_info.y,request.object_info.z)
        object_to_add.task_when_seen = "TODO: Build link with Mission Control"

        print(object_to_add)


        response.object_state.description = object_to_add.description
        
        response.object_state.x = object_to_add.x
        response.object_state.y = object_to_add.y
        response.object_state.z = object_to_add.z
        response.object_state.time_seen = object_to_add.time_seen

        response.object_state.location = object_to_add.location
        response.object_state.task_when_seen = object_to_add.task_when_seen
        
        print(response)

        with self.conn:
            last_row_id = database_functions.create_object(self.conn, object_to_add)

        response.object_state.id = last_row_id
        if response.object_state.description == "":
            response.object_state.description = "Element not found"

        self.get_logger().info(
            'Object:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Task: %s\n  Time: %s'
            %(response.object_state.id, response.object_state.description, response.object_state.location,response.object_state.x,response.object_state.y,response.object_state.z,response.object_state.task_when_seen,str(response.object_state.time_seen)))

        print("response:\n",response,"\n",type(response))
        return response



    def determine_room(self, x: float, y: float, z: float) -> str:
        # Depends on simulation room, since we are assuming we have a map apriori
        # Todo: work with actual simulated home

        if x>0 and x<=10 and y>0 and y<10: # first quadrant (TR)
            return "Living Room"
        elif x>-10 and x<=0 and y>0 and y<10: #second quadrant (TL)
            return "Bathroom"
        elif x>-10 and x<=0 and y>-10 and y<=0: #third quadrant (BL):
            return "Bedroom"
        elif x>0 and x<=10 and y>-10 and y<=0 : #fourth quadrant (BR)
            return "Kitchen"
        else:
            return "outside the house"


        
def main():
    rclpy.init()

    service = SpotNewObjectService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()