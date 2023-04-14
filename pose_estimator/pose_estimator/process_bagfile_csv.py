
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped



class CSVConverter(Node):

    def __init__(self):
        super().__init__('csv_converter')
        self.subscription = self.create_subscription(
            msg_type=PoseStamped,
            topic="vehicle_pose",
            callback= self.callback,
            qos_profile=1)
        self.subscription  # prevent unused variable warning

        self.cnt = 0

        self.fp = open('data.txt', 'w')
        print('UTM E, UTM N', file=self.fp) # Where you need to match what you print later

    def callback(self, msg):

        self.get_logger().info('cnt = "%d' % self.cnt)
        self.cnt = self.cnt + 1
        print('gps count = ', self.cnt)
        print(msg.pose.position.x, ', ', msg.pose.position.y, file = self.fp) # this is where you are printing data

def main(args=None):
    rclpy.init(args=args)

    csv_converter= CSVConverter()

    rclpy.spin(csv_converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    csv_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()