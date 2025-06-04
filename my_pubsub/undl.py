# Imports
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import pigpio
import math
from my_pubsub.read_PWM import reader
# Setup
pi = pigpio.pi()
if not pi.connected:
    exit()

throttle_gpio = 12
turn_gpio = 13
rc_gpio = 18
relay_gpio = 16


# Set the GPIO to servo mode
pi.set_mode(throttle_gpio, pigpio.OUTPUT)
pi.set_mode(turn_gpio, pigpio.OUTPUT)
pi.set_mode(relay_gpio, pigpio.OUTPUT)


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
        
        self.control_active = False
        self.off = True
        self.latest_power = 1500
        self.latest_turn = 1500

        # Run loop every 0.5s
        self.timer = self.create_timer(0.5, self.control_loop)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.position)
        power = msg.position[0]

        if power != -1:
            self.latest_power = power
            self.latest_turn = msg.position[1]
            self.control_active = True
        else:
            self.control_active = False
            self.off = True
            pi.write(relay_gpio, 1)
            pi.set_servo_pulsewidth(throttle_gpio, 1500)
            pi.set_servo_pulsewidth(turn_gpio, 1500)

    def control_loop(self):
        if not self.control_active:
            return  # skip loop if control is off

        freq = r.frequency()
        pulse = r.pulse_width()
        duty = r.duty_cycle()
        print(f"Freq: {freq:.1f} Hz | Pulse: {pulse:.1f} µs | Duty: {duty:.1f}%")

        if round(duty, 1) != 7.5 and not self.off:
            pi.write(relay_gpio, 1)
            print("Remote control being used")
            self.off = True

        elif round(duty, 1) == 7.5 and self.off:
            time.sleep(1)
            if round(r.duty_cycle(), 1) == 7.5:
                time.sleep(1)
                if round(r.duty_cycle(), 1) == 7.5:
                    print("Raspberry Pi in control")
                    pi.write(relay_gpio, 0)
                    self.off = False

        pi.set_servo_pulsewidth(throttle_gpio, self.latest_power)
        pi.set_servo_pulsewidth(turn_gpio, self.latest_turn)



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        print("Shutting down from keyboard interrupt...")
    finally:
        # Stop PWM outputs and relay
        pi.set_servo_pulsewidth(throttle_gpio, 1500)
        pi.set_servo_pulsewidth(turn_gpio, 1500)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(throttle_gpio, 0)
        pi.set_servo_pulsewidth(turn_gpio, 0)
        pi.write(relay_gpio, 1)

        # Clean up pigpio
        r.cancel()
        pi.stop()

        # Clean up ROS 2
        minimal_subscriber.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
