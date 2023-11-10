from capstone_interfaces.srv import ObjectDescriptionQuery, ObjectIDQuery
from capstone_interfaces.msg import StateObject
# import database_functions
import sqlite3

import rclpy
import builtin_interfaces
from rclpy.node import Node


class DescriptionQueryService(Node):

    def __init__(self):
        super().__init__('description_query_service')
        conn = create_connection()
        self.srv = self.create_service(ObjectDescriptionQuery, 'object_description_query', self.object_query)

    def object_query(self, description: str, state_list: list[StateObject]) -> list[StateObject]: # is this accurate? should output be only on inside or outside of parentheses?
        """
        Query objects by description
        : param description
        :return: all 
        """
        conn = create_connection()

        # searches SQLite objects table for the description 
        cur = conn.cursor()
        cur.execute("SELECT * FROM objects WHERE description=?",(description,))
        rows = cur.fetchall()        

        # def read_database_rows(rows) -> StateObject[]: state

        for row in rows:
            state_list[row].id = row[0]
            state_list[row].description = row[1]
            state_list[row].location = row[2]
            state_list[row].x = row[3]
            state_list[row].y = row[4]
            state_list[row].z = row[5]
            state_list[row].task_when_seen = row[6]
            state_list[row].time_seen = row[7] # need to play with datatypes to convert correctly

        self.get_logger().info('Description queried: %d' % (description))

        return state_list


def main():
    rclpy.init()

    description_service = DescriptionQueryService()

    rclpy.spin(description_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()



def create_connection():
    """ create a ddatabase connection to a database that resides in the memory """
    conn = None
    try:
        #conn = sqlite3.connect(':memory:')
        conn = sqlite3.connect(r'state_db.db')
        return conn
    except Error as e:
        print(e)

    return conn

def create_object_table(conn):
    """ create a table from the create_table_sql statement
    :param conn: Connection object
    :param create_table_sql: a CREATE TABLE statement
    :return:
    """
    sql_create_objects_table = """ CREATE TABLE IF NOT EXISTS objects (
                                        id integer PRIMARY KEY,
                                        description text NOT NULL,
                                        location text NOT NULL,
                                        x float NOT NULL,
                                        y float NOT NULL,
                                        z float NOT NULL,
                                        task text,
                                        timestamp text NOT NULL,
                                    ); """
    try:
        c = conn.cursor()
        c.execute(create_table_sql)
    except Error as e:
        print(e)

def create_object(conn, new_object):
    """
    Create a new project into the projects table
    :param conn:
    :param object:
    :return: object id
    """
    sql = ''' INSERT INTO objects(description,location,x,y,z,task,timestamp)
              VALUES(?,?,?,?,?,?,?) '''
    cur = conn.cursor()
    cur.execute(sql, new_object)
    conn.commit()
    return cur.lastrowid