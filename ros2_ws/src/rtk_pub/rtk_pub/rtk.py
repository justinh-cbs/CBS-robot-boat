import rclpy
from rclpy.node import Node

import rtk_pub.rtk_pub as rtk


class RtkNode(Node):
    def __init__(self):
        super().__init__("rtk_node")
        self.node = rclpy.create_node("minimal_client")
        rtk.main()
        self.node.get_logger().info("Started RTK node...")


def main(args=None):
    rclpy.init(args=args)
    rtk_node = RtkNode()
    rclpy.spin(rtk_node)


if __name__ == "__main__":
    main()
