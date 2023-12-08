import sqlite3

from litterbug.items import Item

# Read args to grab csv file and db path
import sys

csv_file = sys.argv[1]
db_file = sys.argv[2]
print(f"csv_file: {csv_file}")
print(f"db_file: {db_file}")

conn = sqlite3.connect(db_file)

items = Item.from_csv(csv_file)

for item in items:
    sql = """ INSERT INTO objects(description,location,x,y,z)
              VALUES(?,?,?,?,?) """
    cur = conn.cursor()

    cur.execute(sql, (item.label, "", item.origin[0], item.origin[1], item.origin[2]))
    conn.commit()
