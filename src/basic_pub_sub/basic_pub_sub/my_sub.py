# Node:
# Written by:
# Date Created: Jan 2023
# Description: Python ROS2 node for converting joystick commands to msgs to send to the controller.

# Import all Necessary Python Modules Here

#import something

# Import all Necessary ROS2 Modules Here, this includes messages (std_msgs.msg for Int16, geometry_msgs.msg for Pose) Look at this
# for all message data: https://av1tenth-docs.readthedocs.io/en/latest/information/ros2_common_msgs.html

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, String


class NodeTemplate(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("node_name")

        # self.subscription = self.create_subscription(msg_type = , topic = , callback = , qos_profile = 1)
        # self.publisher = self.create_publisher(msg_type = , topic = , qos_profile = 1)
        # self.timer = self.create_timer(timer_period_sec = , callback = )

    def callback(self, msg=Int16):
        """Description of Callback"""
        # Do Something

        # self.get_logger().info(" ")
        # self.get_logger().info(f"Something {var}") # Use this if you want to print a variable to the screen

    def callback2(self):
        """Description of Callback"""
        # You need replace this

        # Do Something

        # self.get_logger().info(" ") # Use this if it is just a string
        # self.get_logger().info(f"Something {var}") # Use this if you want to print a variable to the screen


def main(args=None):
    rclpy.init(args=args)
    node_template = NodeTemplate()
    rclpy.spin(node_template)
    node_template.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
