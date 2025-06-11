# Imports
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import pigpio
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'topic',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            # Initialize the pigpio library
            pi = pigpio.pi()

            # Check if pigpio is connected
            if not pi.connected:
               exit()

            # Set the GPIO pin for PWM output (e.g., GPIO 18)
            pwm_pin = 18

            # Set the PWM frequency
            pwm_frequency = 50
            pi.set_PWM_frequency(pwm_pin, pwm_frequency)

            # Set the PWM range (e.g., 1000, for values from 0 to 1000)
            pwm_range = 1000
            pi.set_PWM_range(pwm_pin, pwm_range)
            pi.set_PWM_dutycycle(pwm_pin,50 )
            time.sleep(1)

            self.get_logger().info('I heard: "%s"' % msg.position)
            power = msg.position[0]
            duration = msg.position[1]
            print("Showing " + str(power) + " percent motor power for " + str(duration) + " seconds.")
            # Scaling range
            power = math.ceil((power/100) * 32 + 58)
            pi.set_PWM_dutycycle(pwm_pin, power)
            time.sleep(duration)
            pi.set_PWM_dutycycle(pwm_pin, 0)
            pi.stop()
        except KeyboardInterrupt:
            # Clean up and stop PWM on Ctrl+C
            pi.set_PWM_dutycycle(pwm_pin, 0)
            pi.stop()

def main(args=None):

    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
