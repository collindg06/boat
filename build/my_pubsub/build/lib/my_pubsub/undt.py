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
        noerror = False
        while noerror == False:
            power = int(input("Enter how much power from 1540 (no power) - 1660 (max power) (1540 is neutral):  "))
            if power =! -1:
                turn = int(input("How much do you want to turn from 700 - 2500 (1500 is neutral):  "))
            else:
                turn = 0
            if power < 1540 or power > 1660 or turn < 700 or turn > 2500:
                if (x == -1):
                    msg = JointState()
                    msg.position = power
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: "%s"' % msg.position)
                    takecontrol = input("Enter something to contnue using pi or click nothing to not")
                else:
                    print("Incorrect input")
            else:
                noerror = True
        msg = JointState()
        namelist=[power, turn]
        msg.position =  namelist
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position[0])
        self.get_logger().info('Publishing: "%s"' % msg.position[1])
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

