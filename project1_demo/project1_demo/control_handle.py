import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class control_handler(Node):

    def __init__(self):
        super().__init__('control_handler_node')
        
        # method variables
        self.button_dict = {'A':0, 'B':1, 'X':2, "Y":3, "LB":4, "RB":5, "Back":6, "Start":7, "Xbox":8, "L3":9, "R3":10}
        self.isStopped = False

        # parameter
        self.declare_parameter('stop_button', 'X')

        self.get_logger().info(f"Stop button is: {self.get_parameter('stop_button').get_parameter_value().string_value}")
        
        # define subscriber and publisher
        self.subscriber = self.create_subscription(Joy, 'joy', self.sub_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

    def sub_callback(self, msg : Joy):
      
        # make ackermann msg tand store header
        acker_msg = AckermannDriveStamped()
        acker_msg.header = msg.header

        # set the ROC values to indicate unlimited ROC
        acker_msg.drive.steering_angle_velocity = 0.0
        acker_msg.drive.acceleration = 0.0
        acker_msg.drive.jerk = 0.0

        # check stop button
        my_param = self.get_parameter('stop_button').get_parameter_value().string_value
        index = self.button_dict[my_param]
        stop_button = msg.buttons[index]

        if stop_button and not self.isStopped:
            self.isStopped = True
            self.get_logger().info("Stop button pressed!")
        
        if self.isStopped:
            acker_msg.drive.speed = 0.0 
            acker_msg.drive.steering_angle = 0.0
            self.publisher.publish(acker_msg)
            return

        # if not pressed
        acker_msg.drive.speed = msg.axes[1] * 6
        acker_msg.drive.steering_angle = msg.axes[0] * -45

        #self.get_logger().info(f"Publishing Ackermann: Speed = {acker_msg.drive.speed}, Angle = {acker_msg.drive.steering_angle}")
        self.publisher.publish(acker_msg)


def main(args=None):
    rclpy.init(args=args)

    my_node = control_handler()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        print(" Keyboard Interrupt has occured, exiting program...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()