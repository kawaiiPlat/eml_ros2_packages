#!/usr/bin/env python3

# Node: quotient_publisher
# Written by: Jarrod Sanders (github.com/kawaiiPlat)
# Date Created: Jan 31 2023
# Description: Python ROS 2 node that outputs the quotient for a divisor of 5 but only when the remainder is equal to 0.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, String


class QuotientPublisher(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("quotient_publisher")

        self.sub = self.create_subscription(msg_type = Int16, topic = "rand_num", callback = self.calculate_quotient, qos_profile = 1)
        self.pub = self.create_publisher(msg_type = Float32, topic = "quotient", qos_profile = 1)

    def calculate_quotient(self, msg =Int16):
        """Outputs the quotient for a divisor of 5 but only when the remainder is equal to 0."""
        if(msg.data % 5 == 0):
            newmsg = Float32()
            newmsg.data = msg.data / 5.0
            self.pub.publish(newmsg)
            self.get_logger().info(f"Sent out a quotient of {newmsg.data}")

        # self.get_logger().info(f"incoming: {msg.data}, mod: {msg.data % 5}") # Use this if you want to print a variable to the screen

        # self.get_logger().info(" ")


def main(args=None):
    rclpy.init(args=args)
    qp = QuotientPublisher()
    rclpy.spin(qp)
    qp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
