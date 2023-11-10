import sqlite3


def create_connection():
    """ create a ddatabase connection to a database that resides in the memory """
    conn = None
    try:
        conn = sqlite3.connect(':memory:')
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