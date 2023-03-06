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

        # PID method variables
        self.e_k = []
        self.Sigma = 0

    def sub_callback(self, msg: LaserScan):
               
        # create message
        acker_msg = AckermannDriveStamped()

        # hardcode  all other values 
        acker_msg.header = msg.header
        acker_msg.drive.speed = 1.0
        acker_msg.drive.acceleration = 0.0
        acker_msg.drive.jerk = 0.0
        acker_msg.drive.steering_angle_velocity = 0.0

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

        # get error
        error = d_setpoint - D_perp_L

        # PID constants
        # for now it is "???", they need to be figured out through experimenting
        K_p = -45*0.50  #??? # , lets say 
        K_i = -45*0.25  #???
        K_d = -45*0.05  #???

        # add new error to list
        length = len(self.e_k)
        k = length-1

        """Proportional"""
        P = K_p * error

        """Integration"""
        # window range for integration
        window = 100
        Sum = 0   

        # logic for if list length is less than desired
        if length < window:
            # add new error value to list and 
            # sigma method var
            self.e_k.append(error)
            self.Sigma += error

            # update length and k
            length += 1
            k  += 1
        else:
            # remove first error in list from 
            # sigma method var and list
            # add new error
            self.Sigma += error - self.e_k[0]
            self.e_k.pop(0)
            self.e_k.append(error)
            Sum = self.Sigma

        I = K_i * Sum * 0.1

        """Derivative"""
        # for now, we will only be using a PI controller
        # this is if we want to test 
        D = 0 
        # if length > 1:
        #   D = K_d * (self.e_k[k]-self.e_k[k-1])/0.1 

        # steering angle
        u = (P + I + D)

        #u = u/45


        # store as steering angle
        acker_msg.drive.steering_angle = math.radians(u)

        self.get_logger().info(f"{D_perp_L}")
        self.get_logger().info(f"{P}")
        self.get_logger().info(f"{I}")
        self.get_logger().info(f"{D}")
        self.get_logger().info(f"{u}")
        self.get_logger().info(f"\n")
        # publish msg
        self.publisher_.publish(acker_msg)
                
def main(args=None):
    rclpy.init(args=args)

    my_node = BangBangController()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt():
        print(" Keyboard Interrupt has occured. Exiting Program now.")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()