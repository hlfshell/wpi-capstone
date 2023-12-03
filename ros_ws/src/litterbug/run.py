from litterbug.map import Map
from litterbug.litterbug import Litterbug
from litterbug.items import Item
import rclpy
from time import sleep

rclpy.init()

# map = Map([])
# rclpy.spin(map)

# map = Map.FromMapFile("tbw4")
# map = Map.FromMapFile("map")
# map = Map.FromMapFile("tb_test")
# map = Map.FromMapFile("remap", (-2.0, 0.5))
map = Map.FromMapFile("remap", (-0.5, -2.0))
# map.FromOccupancy()


items = Item.from_csv("./test.csv")

print("creating litterbug")
lb = Litterbug(
    # [
    #     Item("can", "can", (-0.052102, 1.827700, 0.5), (0, 0, 0, 0)),
    #     # Item("can2", "can2", (1.8277, -0.052102, 0.5), (0, 0, 0, 0)),
    #     Item("can2", "can2", (-0.062137, -1.826920, 0.5), (0, 0, 0, 0)),
    # ],
    items,
    map,
)
lb.wait_for_ready()
lb.populate()

# print(items[0])

# rclpy.spin(lb)

# while True:
#     sleep(1.0)
#     try:
#         lb.__pickup_object()

rclpy.spin(lb)
