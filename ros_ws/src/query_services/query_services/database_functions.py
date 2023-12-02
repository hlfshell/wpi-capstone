import sqlite3
from capstone_interfaces.msg import StateObject
from rclpy.node import Node

import datetime
import calendar

def create_connection(node:Node,db_file:str)->sqlite3.Connection:
    """ create a database connection """
    node.get_logger().info("made connection") 
    conn = None
    try:
        conn = sqlite3.connect(db_file)
        return conn
    except Exception as e:
        node.get_logger().error(e)

    return conn

def create_object_table(conn:sqlite3.Connection, node:Node):
    """ create a table from the create_table_sql statement
    :param conn: Connection object
    :param create_table_sql: a CREATE TABLE statement
    :return:
    """
    sql_create_objects_table = """CREATE TABLE IF NOT EXISTS objects (
                                        id integer PRIMARY KEY,
                                        description text NOT NULL,
                                        location text NOT NULL,
                                        x float NOT NULL,
                                        y float NOT NULL,
                                        z float NOT NULL,
                                        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                                );"""
                                    
    c = conn.cursor()
    c.execute(sql_create_objects_table)
    node.get_logger().info("Table created")

def create_object(conn:sqlite3.Connection, new_object:StateObject) -> int:
    """
    Create a new project into the projects table
    :param conn:
    :param object:
    :return: object id
    """
    description = new_object.description
    location = new_object.location
    x = new_object.x
    y = new_object.y
    z = new_object.z
    s = new_object.time_seen.sec
    ns = new_object.time_seen.nanosec
    timestamp_ingested = s + (ns*1e-9)
    new_obj = (description,location,x,y,z,timestamp_ingested);
    sql = ''' INSERT INTO objects(description,location,x,y,z,timestamp)
              VALUES(?,?,?,?,?,datetime(?,'unixepoch')) '''
    cur = conn.cursor()
    cur.execute(sql, new_obj)
    conn.commit()
    return cur.lastrowid

def datetime2epoch(datetime_str:str)->int:
    date = datetime_str.split()[0]
    time = datetime_str.split()[1]
    year = date.split("-")[0]
    month = date.split("-")[1]
    day = date.split("-")[2]

    hour = time.split(":")[0]
    minutes = time.split(":")[1]
    seconds = time.split(":")[2]

    t = datetime.datetime(int(year),int(month),int(day),int(hour),int(minutes),int(seconds))

    return calendar.timegm(t.timetuple())

def update_task(conn:sqlite3.Connection, update_object:StateObject):
    """
    update priority, begin_date, and end date of a task
    :param conn:
    :param task:
    :return: project id
    """
    description = update_object.description
    location = update_object.location
    x = update_object.x
    y = update_object.y
    z = update_object.z
    s = update_object.time_seen.sec
    ns = update_object.time_seen.nanosec
    timestamp_ingested = s + (ns*1e-9)
    update_object = (description,location,x,y,z,timestamp_ingested);

    sql = ''' UPDATE objects
              SET description = ? ,
                  location = ? ,
                  x = ?
                  y = ?
                  z = ?
                  timestamp = ?
              WHERE id = ?'''
    cur = conn.cursor()
    cur.execute(sql, update_object)
    conn.commit()