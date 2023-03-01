import rclpy
from rclpy.node import Node

import math
import numpy as numpy

from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from ackermann_msgs.msg import AckermannDriveStamped


class BangBangController(Node):

    def __init__(self):
        super().__init__('BangBangControllerNode')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)
        self.subcriber = self.create_subscription(LaserScan, 'scan', self.sub_callback, 10)

    def sub_callback(self, msg: LaserScan):
        pass
        
        acker_msg = AckermannDriveStamped()

        acker_msg.header = msg.header
        acker_msg.drive.speed = 1.0
        acker_msg.drive.acceleration = 0.0
        acker_msg.drive.jerk = 0.0
        acker_msg.drive.steering_angle_velocity = 0.0

        d_setpoint = 0.5
        angle = 30.0
        coeffient = 1

        d_offset = msg.range[(90+angle)*msg.angle_increment]

        if d_offset > d_setpoint:
            coeffient = 1
        else :
            coeffient = -1
        
        acker_msg.drive.steering_angle = math.radians(coeffient*10)

        self.publisher_.publish(acker_msg)
            
def main(args=None):
    rclpy.init(args=args)

    my_node = BangBangController()

    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()