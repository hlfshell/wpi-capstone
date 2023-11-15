from capstone_interfaces.srv import ObjectDescriptionQuery, ObjectIDQuery
from capstone_interfaces.msg import StateObject
from query_services import database_functions

# from ros_ws.src.query_services import database_functions
import sqlite3

import rclpy
import builtin_interfaces
from rclpy.node import Node


class DescriptionQueryService(Node):

    def __init__(self):
        super().__init__('description_query_service')
        self.conn = database_functions.create_connection(r"state_db.db")
        self.srv = self.create_service(ObjectDescriptionQuery, 'object_description_query', self.object_query)

    def object_query(self, description: str, response: list[StateObject]) -> list[StateObject]:
        """
        Query objects by description, recieves description, queries db, and resturns a list of state objects to the service
        """   

        # searches SQLite objects table for the description 
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM objects WHERE description=?",(description.object_description,))
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
            curr_state.task_when_seen = row[6]
            time_str = row[7]
            s_ms=(time_str.split("(")[1]).split(",")
            print("s_ms",s_ms)
            s = int(s_ms[0].split("=")[1])
            ms = int((s_ms[1].split("=")[1]).strip(")"))
            print("s",s,"ms",ms)

            time_obj = builtin_interfaces.msg.Time()
            time_obj.sec = s
            time_obj.nanosec = ms
            curr_state.time_seen = time_obj

            state_list.append(curr_state)

        self.get_logger().info('Description queried: %s' % (description))
        print("Description Queried: ",description)
        if rows:
            print("Object(s): \n",state_list)
            response.states_of_objects = state_list
        else:
            print("No results found")
            state_list_0 = StateObject()
            state_list_0.description = "No results found"
            response.states_of_objects = [state_list_0.description]

        return response


def main():
    rclpy.init()

    description_query_service = DescriptionQueryService()

    rclpy.spin(description_query_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

