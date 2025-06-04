# Imports
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import pigpio
import math
from read_PWM import reader
# Setup
pi = pigpio.pi()
if not pi.connected:
    exit()

throttle_gpio = 12
turn_gpio = 13
rc_gpio = 18
relay_gpio = 14


# Set the GPIO to servo mode
pi.set_mode(throttle_gpio, pigpio.OUTPUT)
pi.set_mode(turn_gpio, pigpio.OUTPUT)

# Read PWM on GPIO18
r = reader(pi, rc_gpio)  

# Set relay to pi
pi.write(relay_gpio,0)

pi.set_servo_pulsewidth(throttle_gpio, 1500)
pi.set_servo_pulsewidth(turn_gpio, 1500)
time.sleep(2)

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
            self.get_logger().info('I heard: "%s"' % msg.position)
            power = msg.position[0]
            turn = msg.position[1]
            while True:
                print("Freq: {:.1f} Hz | Pulse: {:.1f} µs | Duty: {:.1f}%".format(
                    r.frequency(), r.pulse_width(), r.duty_cycle()))
                if r.duty_cycle() >  8.5:
                    pi.write(relay_gpio,1)
                    print("Remote control being used")
                if round(r.duty_cycle(), 1) == 8.4:
                    time.sleep(1)
                    if round(r.duty_cycle(), 1)  == 8.4:
                        time.sleep(1)
                        if round(r.duty_cycle(), 1)  == 8.4:
                            print("Rasberry pi in control")
                            pi.write(relay_gpio,0)
                pi.set_servo_pulsewidth(throttle_gpio, power)
                pi.set_servo_pulsewidth(turn_gpio, turn)
                time.sleep(0.5)
                
                
        except KeyboardInterrupt:
            pi.set_servo_pulsewidth(throttle_gpio, 1500)
            pi.set_servo_pulsewidth(turn_gpio, 1500)
            time.sleep(.5)
            # Cleanup
            pi.set_servo_pulsewidth(throttle_gpio, 0)
            pi.set_servo_pulsewidth(turn_gpio, 0)
            r.cancel()
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
