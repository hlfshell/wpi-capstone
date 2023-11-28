from litterbug.map import Map
from litterbug.litterbug import Litterbug
from litterbug.items import Item
import rclpy

rclpy.init()

# map = Map([])
# rclpy.spin(map)

# map = Map.FromMapFile("tbw4")
# map = Map.FromMapFile("map")
# map = Map.FromMapFile("tb_test")
map = Map.FromMapFile("remap")
# map.FromOccupancy()

print("creating litterbug")
lb = Litterbug(
    [
        Item("can", "can", (0.026827, 1.776700, 0.5), (0, 0, 0, 0)),
        Item("can2", "can2", (-0.062137, -1.826920, 0.5), (0, 0, 0, 0)),
    ],
    map,
)
# lb.test()
rclpy.spin(lb)
