import rclpy
from rclpy.node import Node


from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np



class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__("occupancy_grid_pub_node")
        self.grid_publisher_ = self.create_publisher(OccupancyGrid, "map", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.og_pub_callback)

    def og_pub_callback(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map_frame"

        occupancy_grid.info.resolution = 1.0
        occupancy_grid.info.width = 3
        occupancy_grid.info.height = 3

        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0

        array = np.array([1, 1, 0, 0, 0, 1, 1, 1, 1], dtype=np.int8).tolist()
        occupancy_grid.data = array

        self.grid_publisher_.publish(occupancy_grid)


def main(args=None):
    rclpy.init(args=args)

    publisher = OccupancyGridPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
