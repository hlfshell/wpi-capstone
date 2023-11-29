from litterbug.gazebo import Gazebo
import rclpy

rclpy.init()

gazebo = Gazebo()
gazebo.wait_for_ready()

gazebo.generate_csv("./out.csv")
