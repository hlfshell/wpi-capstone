import rclpy
from explorer.service import SearchService


def main(args=None):
    rclpy.init(args=args)

    node = SearchService()
    # node.test()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
