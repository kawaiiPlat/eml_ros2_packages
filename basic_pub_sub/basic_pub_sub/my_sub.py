# Node:
# Written by:
# Date Created: Jan 2023
# Description: Python ROS2 node for converting joystick commands to msgs to send to the controller.

# Import all Necessary Python Modules Here

#import something

# Import all Necessary ROS2 Modules Here, this includes messages (std_msgs.msg for Int16, geometry_msgs.msg for Pose) Look at this
# for all message data: https://av1tenth-docs.readthedocs.io/en/latest/information/ros2_common_msgs.html

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3


class cartToCirc(Node):
    """Translate cartesian to Circular coords"""

    def __init__(self):
        super().__init__("my_sub_pub")

        self.subscription = self.create_subscription(
            msg_type = Vector3,
            topic = "pub_me",
            callback = self.calc_circular,
            qos_profile = 1
        )
        self.publisher = self.create_publisher(
            msg_type = Float64MultiArray,
            topic = "lets_go",
            qos_profile = 1
        )
        # self.timer = self.create_timer(timer_period_sec = , callback = )

    def calc_circular(self, msg=Vector3):
        """Calculate circular coords from cartesian coords which are in the x, y of the vec3"""

        r = math.sqrt(msg.x * msg.x + msg.y + msg.y)

        sin_angle = msg.y / r
        cos_angle = msg.x / r
        theta = math.atan2(cos_angle, sin_angle)

        out = Float64MultiArray()


        self.publisher()
        self.get_logger().info(f"recieved {msg}")


def main(args=None):
    rclpy.init(args=args)
    node_template = cartToCirc()
    rclpy.spin(node_template)
    node_template.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
