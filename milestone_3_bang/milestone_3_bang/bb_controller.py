import rclpy
from rclpy.node import Node

import math
import numpy as numpy

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class BangBangController(Node):

    def __init__(self):
        super().__init__('BangBangControllerNode')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)
        self.subcriber = self.create_subscription(LaserScan, 'scan', self.sub_callback, 10)

    def sub_callback(self, msg: LaserScan):

        # create message
        acker_msg = AckermannDriveStamped()

        # hardcode  all other values 
        acker_msg.header = msg.header
        acker_msg.drive.speed = 1.0
        acker_msg.drive.acceleration = 0.0
        acker_msg.drive.jerk = 0.0
        acker_msg.drive.steering_angle_velocity = 0.0

        # Old algorithm
        # # set the constants
        # d_setpoint = 0.5
        # angle = 30
        # coeffient = 1

        # # get the distance based on constants
        # d_offset = msg.ranges[(90+angle)*2]

        # # check for error to set coeffient
        # if d_offset > d_setpoint:
        #     coeffient = 1
        # else :
        #     coeffient = -1
        
        # # update steering angle based on coeffient
        # acker_msg.drive.steering_angle = math.radians(coeffient*10)

        # New Algorithm
        # set the constants
        d_setpoint = 0.5
        angle = 30
        coeffient = 1
        L = 0.020

        # get the distances based on constants
        d_offset = msg.ranges[(90+angle)*2]
        d = msg.ranges[90*2]

        # find alpha
        alpha = math.atan2(d_offset*math.cos(math.radians(angle))-d, d_offset*math.sin(math.radians(angle)))

        # find D_perp
        D_perp = d * math.cos(alpha)

        # find D_perp+L
        D_perp_L = L*math.sin(alpha) + D_perp

        # check error
        if D_perp_L > d_setpoint:
            coeffient = 1
        else :
            coeffient = -1
        
        # set steering angle based on coeffient
        acker_msg.drive.steering_angle = math.radians(coeffient*10)

        # publish
        self.publisher_.publish(acker_msg)
            
def main(args=None):
    rclpy.init(args=args)

    my_node = BangBangController()

    try:
       rclpy.spin(my_node)
    except KeyboardInterrupt():
        print(" Keyboard Interrupt has occured. Exiing program now.")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()