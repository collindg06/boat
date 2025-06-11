from random import randint
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic', 10)
        timer_period = .5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        power = float(randint(0,100))
        duration = 5.00
        msg = JointState()
        namelist=[power, duration]
        msg.position =  namelist
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position[0])
        self.get_logger().info('Publishing: "%s"' % msg.position[1])
        time.sleep(duration)
        self.i += 1



def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

