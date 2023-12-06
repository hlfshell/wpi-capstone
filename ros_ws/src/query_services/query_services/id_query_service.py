import sqlite3

import builtin_interfaces
import rclpy
from capstone_interfaces.msg import StateObject
from capstone_interfaces.srv import ObjectIDQuery
from query_services import database_functions
from rclpy.node import Node


class IDQueryService(Node):

    def __init__(self):
        super().__init__('id_query_service')
        self.conn = database_functions.create_connection(self,r"state_db.db")
        self.srv = self.create_service(ObjectIDQuery, 'object_id_query', self.object_query)

    def object_query(self, obj_id: int, response: list[StateObject]) -> list[StateObject]:
        """
        Query objects by id: recieves id, queries db, and resturns a list of state objects to the service
        """   

        # searches SQLite objects table for the id 
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM objects WHERE id=?",(obj_id.object_id,))
        rows = cur.fetchall()        

        state_list = []
        curr_state = StateObject()

        for row in rows:

            curr_state.id = row[0]
            curr_state.description = row[1]
            curr_state.location = row[2]
            curr_state.x = row[3]
            curr_state.y = row[4]
            curr_state.z = row[5]
            datetime_str = row[7] # needs to change when db is updated
            
            epoch = database_functions.datetime2epoch(datetime_str)
            s = round(epoch)
            ms = int((epoch-s)/1e-9)

            time_obj = builtin_interfaces.msg.Time()
            time_obj.sec = s
            time_obj.nanosec = ms
            curr_state.time_seen = time_obj

            state_list.append(curr_state)

        self.get_logger().info('ID queried: %s' % (obj_id))
        if rows:
            self.get_logger().info("Object(s): \n"+str(state_list))
            response.states_of_objects = state_list
        else:
            self.get_logger().info("No results found")
            state_list_0 = StateObject()
            state_list_0.description = "No results found"
            response.states_of_objects = [state_list_0.description]

        return response


def main():
    rclpy.init()

    id_service = IDQueryService()

    rclpy.spin(id_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

