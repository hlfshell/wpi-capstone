from litterbug.map import Map
from litterbug.litterbug import Litterbug
import rclpy

rclpy.init()

map = Map.FromMapFile("tbw4")
map.draw(size=800)

lb = Litterbug([], map)
