import sqlite3
from capstone_interfaces.msg import StateObject


def create_connection(db_file:str)->sqlite3.Connection:
    """ create a database connection """
    print("made connection")
    conn = None
    try:
        conn = sqlite3.connect(db_file)
        return conn
    except Error as e:
        print(e)

    return conn

def create_object_table(conn:sqlite3.Connection):
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
                                        task text,
                                        timestamp text NOT NULL
                                );"""
                                    
    #try:
    c = conn.cursor()
    c.execute(sql_create_objects_table)
    # except:
    #    print("Something went wrong")
    # except OSError as e:
    #    print(e)

def create_object(conn:sqlite3.Connection, new_object:StateObject):
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
    task = new_object.task_when_seen
    timestamp = str(new_object.time_seen)
    new_obj = (description,location,x,y,z,task,timestamp);
    sql = ''' INSERT INTO objects(description,location,x,y,z,task,timestamp)
              VALUES(?,?,?,?,?,?,?) '''
    cur = conn.cursor()
    cur.execute(sql, new_obj)
    conn.commit()
    return cur.lastrowid