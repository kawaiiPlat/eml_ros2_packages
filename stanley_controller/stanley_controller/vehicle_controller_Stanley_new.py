import rclpy
from rclpy.node import Node

import math
import numpy as np


from geometry_msgs.msg import PoseStamped, Twist
from gps_nav_interfaces.msg import CurrentGoalPose
from ackermann_msgs.msg import AckermannDriveStamped
from gps_nav.uf_support.route_support import get_cross_track_and_heading_error

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


class VehicleControllerTemplate(Node):
    def __init__(self):
        super().__init__("vehicle_controller")
        self.subscription1 = self.create_subscription(PoseStamped, "vehicle_pose", self.vehicle_pose_callback, 1)

        self.subscription2 = self.create_subscription(CurrentGoalPose, "current_goal_pose", self.current_goal_pose_callback, 1)

        self.publisher = self.create_publisher(AckermannDriveStamped, "vehicle_command_ackermann", 10)
        # self.publisher2 = self.create_publisher(Twist, "vehicle_command_twist", 10)

        # set up the timer (0.1 sec) to send over the current_carrot message to the vehicle controller
        self.main_timer = self.create_timer(0.1, self.main_timer_callback)


        # define the variables that will store the data from the two message inputs
        self.current_goal_point = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad = 0.0
        self.closest_point = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad = 0.0
        self.speed = 0.0
        self.state = 0.0
        self.goal_cnt = 0

        self.vehicle_point = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad = 0.0
        self.vehicle_pose_cnt = 0

        self.have_vehicle_pose = False
        self.have_goal_pose = False

        self.old_steering_angle = 0.0
        self.prev_error = 0.0
        self.t_current = 0
        self.t_previous = 0
        self.error_integral = 0.0

        self.xvec = np.array([1.0, 0.0, 0.0])
        self.yvec = np.array([0.0, 1.0, 0.0])
        self.zvec = np.array([0.0, 0.0, 1.0])

        self.K = 0.25

    def vehicle_pose_callback(self, msg):
        self.have_vehicle_pose = True

        self.vehicle_point = np.array([0.0, 0.0, 0.0])
        self.vehicle_point[0] = msg.pose.position.x
        self.vehicle_point[1] = msg.pose.position.y
        self.vehicle_point[2] = 0.0
        self.vehicle_heading_rad = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

        self.vehicle_pose_cnt += 1

    def current_goal_pose_callback(self, msg):
        self.have_goal_pose = True

        self.current_goal_point = np.array([0.0, 0.0, 0.0])
        self.current_goal_point[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point[2] = 0.0
        self.current_goal_heading_rad = 2.0 * math.atan2(msg.current_goal_pose.pose.orientation.z, msg.current_goal_pose.pose.orientation.w)

        self.closest_point = np.array([0.0, 0.0, 0.0])
        self.closest_point[0] = msg.closest_pose.pose.position.x
        self.closest_point[1] = msg.closest_pose.pose.position.y
        self.closest_point[2] = 0.0
        self.closest_heading_rad = 2.0 * math.atan2(msg.closest_pose.pose.orientation.z, msg.closest_pose.pose.orientation.w)

        self.speed = msg.speed
        self.state = msg.state

        self.goal_cnt += 1

    def main_timer_callback(self):
        # will only publish after each of the subscribes has occured at least once

        if not (self.have_goal_pose and self.have_vehicle_pose):  # don't touch this
            return
        # Put your controller below here.            
        error_cross_track, error_headng_rad, line_pt = get_cross_track_and_heading_error(self.closest_point, self.closest_heading_rad, self.vehicle_point, self.vehicle_heading_rad)

        steering_angle_rad = error_headng_rad + math.atan2(self.K*error_cross_track,1.25)

        if steering_angle_rad > math.radians(45):
            steering_angle_rad = math.radians(45)
        if steering_angle_rad < math.radians(-45):
            steering_angle_rad = math.radians(-45)

        out_msg = AckermannDriveStamped()
        out_msg.drive.speed = 1.5
        out_msg.drive.steering_angle = steering_angle_rad

        # now send a VehCmd message out
        # out_msg = VehCmd()
        # out_msg.steering_angle = 0.0
        # if self.speed < 0.01:
        #     out_msg.throttle_effort = 0.0
        # else:
        #     out_msg.throttle_effort = 50.0  # percent

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    my_vehicle_controller = VehicleControllerTemplate()

    rclpy.spin(my_vehicle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
