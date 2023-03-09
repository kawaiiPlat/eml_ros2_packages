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
        d_setpoint = 0.50
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

        # normalize the error 
        error /= d_setpoint 

        # PID constants
        # for now it is "???", they need to be figured out through experimenting
        K_p = -1.50    #-1*0.50  #??? # , lets say 
        K_i =  1*0.35  #???
        K_d = -1*0.05  #???

        # window length
        window = 100

        # add new error to list
        length = len(self.e_k)
        k = length-1

        # Check if NaN or inf
        if (math.isnan(D_perp_L) or math.isinf(D_perp_L)):
            if (length == 0):
                error = 0
            else:
                error = self.e_k[k]
        
        # check if length less than the window
        if (length < window):
            self.e_k.append(error)
            length += 1
            k += 1
            self.Sigma += error
        else: 
            oldest_error = self.e_k.pop(0)
            self.Sigma -= oldest_error
            self.Sigma += error
            self.e_k.append(error)
            

        """Proportional"""
        P = K_p * error

        """Integration"""
        # window range for integration
        # Sum = 0   

        # if (math.isnan(d_offset) or math.isinf(d_offset) or math.isnan(d) or math.isinf(d)):
            
        #     error = self.e_k[len(self.e_k)-1]

        #     # Check if Window is Full.
        #     if (length >= window) :

        #         # Pop Oldest Value from Integration Window.
        #         oldest = self.e_k.pop(0)
        #         self.Sigma -= oldest
        #         self.Sigma += error
        #         self.e_k.append(error)
            
        #     # Check if Window is not Full.
        #     else :
                
        #         if (len(self.e_k)-1 >= 0):

        #             self.e_k.append(error)
        #             self.Sigma += error

        #         else:

        #             return

        # else:

        #     # Check if Window is Full.
        #     if (length >= window) :

        #         # Pop Oldest Value from Integration Window.
        #         oldest = self.e_k.pop(0)
        #         self.Sigma -= oldest
        #         self.Sigma += error
        #         self.e_k.append(error)
            
        #     # Check if Window is not Full.
        #     else :
            
        #         self.e_k.append(error)
        #         self.Sigma += error
 
        # if length < window:
        #     # add new error value to list and 
        #     # sigma method var
        #     self.e_k.append(error)
        #     self.Sigma += error
        #     # Sum = Self.Sigma

        #     # update length and k
        #     length += 1
        #     k  += 1
        # else:
        #     # remove first error in list from 
        #     # sigma method var and list
        #     # add new error
        #     self.Sigma += error - self.e_k[0]
        #     self.e_k.pop(0)
        #     self.e_k.append(error)
        #     # Sum = self.Sigma

        # I = K_i * Sum * 0.1
        I = (K_i * self.Sigma * 0.1) / len(self.e_k)

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
        acker_msg.drive.steering_angle = u #math.radians(u)

        self.get_logger().info(f"D_perp_L: {D_perp_L}")
        self.get_logger().info(f"Error: {error}")
        self.get_logger().info(f"P: {P}")
        self.get_logger().info(f"I: {I}")
        self.get_logger().info(f"D: {D}")
        self.get_logger().info(f"u: {u}")
        self.get_logger().info(f"\n")
        # publish msg
        self.publisher_.publish(acker_msg)
                
def main(args=None):
    rclpy.init(args=args)

    my_node = BangBangController()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.get_logger().info(" Keyboard Interrupt has occured. Exiting Program now.")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()