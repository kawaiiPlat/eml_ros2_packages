#!/usr/bin/env python3


# Node: laserToPC
# Written by: Jarrod Sanders (github.com/kawaiiPlat)
# Date Created: Feb 10 2023
# Description: Python ROS 2 node that converts a LaserScan message to a PointCloud message

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

class LaserToPC(Node):
    """
    Convert a LaserScan to a PointCloud
    There are some points that show up on the pointcloud that are not in the laserscan in rviz.
    I think these points are outside of the min_range and max_range of the laserscan, but the
    assignment didn't specify to filter these out.
    """

    def __init__(self):
        super().__init__("laser_to_pc")
        self.sub = self.create_subscription(msg_type = LaserScan, topic = "scan", callback = self.convert, qos_profile = 1)
        self.pub = self.create_publisher(msg_type = PointCloud, topic = "cart_coord", qos_profile = 1)

    def convert(self, msg =LaserScan):
        """Convert a LaserScan in to a PointCloud"""
        deltaAngle = 0.5 / 360 * np.pi * 2
        startingAngle = 180 / 360 * np.pi * 2
        outcoords = PointCloud()
        outcoords.header = msg.header #pass along the header
        #could add a channels channel with the distance passed along, because why not. It's not like ROS is a bandwi
        for i, d in enumerate(msg.ranges):
            a = i * deltaAngle + startingAngle

            newpt = Point32()
            newpt.x = 1.0 * d * np.cos(a)
            newpt.y = 1.0 * d * np.sin(a)
            newpt.z = 0.0 # could increase this based on geometry of car, not worrying about that right now
            outcoords.points.append(newpt)

        self.pub.publish(outcoords)
        self.get_logger().info(f"Sent out a PointCloud from frame id {msg.header.frame_id}")

        # self.get_logger().info(f"incoming: {msg.data}, mod: {msg.data % 5}") # Use this if you want to print a variable to the screen

        # self.get_logger().info(" ")


def main(args=None):
    rclpy.init(args=args)
    ltp = LaserToPC()
    rclpy.spin(ltp)
    ltp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
