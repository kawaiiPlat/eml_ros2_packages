# Program: Random number generator
# Written by: Aditya Penumarti
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: August
# Description: Random number generator.

from random import randint
from rclpy.node import Node
from std_msgs.msg import Int16
import rclpy


class RandNum(Node):
    def __init__(self):

        super().__init__('rand_num')
        self.pub = self.create_publisher(Int16, 'rand_num', 10)
        timer_period = .5  # seconds
        self.timer = self.create_timer(timer_period, self.rand_callback)
        self.count = 0

    def rand_callback(self):
        msg = Int16()
        rand_int = randint(0, 1000)
        val = rand_int % 5
        if val != 0:
            self.count += 1
        if self.count == randint(0, 10):
            rand_int = min(5*round(rand_int/5), 1000)
            self.count = 0
        msg.data = rand_int
        self.get_logger().info('Number = "%4i"' % rand_int)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rand_num = RandNum()

    rclpy.spin(rand_num)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rand_num.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
