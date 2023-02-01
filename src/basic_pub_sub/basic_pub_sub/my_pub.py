#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class PubVec3(Node):
    def __init__(self):
        super().__init__('my_pub')
        self.publisher_ = self.create_publisher(Vector3, 'pub_me', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.clock = Clock()
        self.startTime = self.clock.now()

    def timer_callback(self):
        msg = Vector3()
        currTime = self.clock.now() - self.startTime
        currTimeSecs = math.floor(currTime.nanoseconds / 1000000000)
        self.get_logger().info(f"{currTimeSecs}")
        msg.x = 5.0 * math.cos(0.2 * currTimeSecs + math.pi / 2)
        msg.y = 8.0 * math.cos(0.6 * currTimeSecs)
        msg.z = 0.05 * math.cos(0.2 * currTimeSecs + math.pi / 2)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')


def main(args=None):
    rclpy.init(args=args)

    vec_pub = PubVec3()

    rclpy.spin(vec_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vec_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
