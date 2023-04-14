from rclpy.node import Node
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi, atan2
from time import time # maybe?
import utm


class Pose_Estimator(Node):
    def __init__(self):

        # initializa node
        super().__init__('Pose_Estimator')

        # Pub/Subs
        self.pub = self.create_publisher(PoseStamped, 'vehicle_pose', 10)
        self.sub1 = self.create_subscription(Odometry, "odometry", self.odometry_callback, 10)
        self.sub2 = self.create_subscription(NavSatFix, "gps", self.gps_callback, 10)

        # timer function every 20 Hz
        timer_period = 1.0/5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # default method variable values
        self.x = 0.0
        self.y = 0.0
        self.w = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.velocity = 0.0

    def odometry_callback(self, msg):

        # get w and z
        self.w = msg.pose.orientation.w
        self.z = msg.pose.orientation.z

        # get velocity
        self.velocity = msg.twist.linear.x

        # get quaternion angle
        self.theta = 2*atan2(self.z,self.w)

        # estimate next pose
        self.x = self.x + self.velocity * cos(self.theta) * 0.15
        self.y = self.y + self.velocity * sin(self.theta) * 0.15

    def gps_callback(self, msg):

        # update x and y with "actual" gps values
        self.x, self.y, _, _ = utm.from_latlon(msg.latitude,msg.longitude)
        pass
        
    def timer_callback(self):

        # store values        
        msgEst = PoseStamped()
        msgEst.pose.position.x = self.x
        msgEst.pose.position.y = self.y
        msgEst.pose.orientation.z = self.z
        msgEst.pose.orientation.w = self.w
        msgEst.header.stamp = self.get_clock().now().to_msg()

        # set the rest to 0 
        msgEst.pose.position.z = 0.0
        msgEst.pose.orientation.x = 0.0
        msgEst.pose.orientation.y = 0.0

        self.pub.publish(msgEst)
    


def main(args=None):
    rclpy.init(args=args)
    my_node = Pose_Estimator()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt has occured! Exiting Program now.\n")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()