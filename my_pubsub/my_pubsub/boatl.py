import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pigpio
import time
from my_pubsub.read_PWM import reader

# === GPIO CONFIGURATION ===
throttle_gpio = 12
turn_gpio = 13
relay1_gpio = 20
relay2_gpio = 21
rc_gpio = 18

# === CONSTANTS ===
neutral = 1500
throttle = neutral
turn = neutral
step = 20
turn_step = 10
throttle_min = 1540
throttle_max = 1660
turn_min = 700
turn_max = 2000

# === INITIALIZE pigpio ===
pi = pigpio.pi()
if not pi.connected:
    print("Cannot connect to pigpio daemon")
    exit()

for gpio in [throttle_gpio, turn_gpio, relay1_gpio, relay2_gpio]:
    pi.set_mode(gpio, pigpio.OUTPUT)

pi.write(relay1_gpio, 0)
pi.write(relay2_gpio, 0)

# === Initialize PWM Reader ===
r = reader(pi, rc_gpio)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            JointState,
            'cmd_motor',
            self.listener_callback,
            10)

        self.throttle = neutral
        self.turn = neutral
        self.cruise_mode = False
        self.cruise_throttle = neutral
        self.last_throttle_input = time.time()
        self.last_turn_input = time.time()
        self.off = True
        self.control_active = False

        self.timer = self.create_timer(0.05, self.update_motors)

    def listener_callback(self, msg):
        if not msg.name:
            return
        key = msg.name[0]

        if key == 'w' and not self.cruise_mode:
            self.throttle = min(self.throttle + step, throttle_max)
            self.last_throttle_input = time.time()
            self.cruise_mode = False
            self.control_active = True

        elif key == 's':
            self.throttle = max(self.throttle - step, neutral)
            self.cruise_mode = False
            self.last_throttle_input = time.time()
            self.control_active = True

        elif key in ['b', 'B']:
            self.throttle = neutral
            self.cruise_mode = False
            self.control_active = False
            self.off = True

        elif key == 'q':
            self.throttle = neutral
            self.turn = neutral
            self.cruise_mode = False
            self.control_active = False

        elif key == 'r':
            relay_state = not pi.read(relay1_gpio)
            pi.write(relay1_gpio, int(relay_state))
            pi.write(relay2_gpio, int(relay_state))

        elif key == 'c':
            if not self.cruise_mode:
                self.cruise_throttle = self.throttle
                self.cruise_mode = True
            else:
                self.cruise_mode = False

        elif key == 'a':
            self.turn = max(self.turn - turn_step, turn_min)
            self.last_turn_input = time.time()
            self.control_active = True

        elif key == 'd':
            self.turn = min(self.turn + turn_step, turn_max)
            self.last_turn_input = time.time()
            self.control_active = True

    def update_motors(self):
        freq = r.frequency()
        pulse = r.pulse_width()
        duty = r.duty_cycle()
        print(f"Freq: {freq:.1f} Hz | Pulse: {pulse:.1f} us | Duty: {duty:.1f}%")

        # Treat both 7.5% and 0.0% duty as Pi control
        if round(duty, 1) not in [7.5, 0.0] and not self.off:
            print("Remote control in use")
            pi.write(relay1_gpio, 1)
            pi.write(relay2_gpio, 1)
            self.off = True

        elif round(duty, 1) in [7.5, 0.0] and self.off:
            time.sleep(0.5)
            if round(r.duty_cycle(), 1) in [7.5, 0.0]:
                time.sleep(0.5)
                if round(r.duty_cycle(), 1) in [7.5, 0.0]:
                    print("Pi back in control")
                    pi.write(relay1_gpio, 0)
                    pi.write(relay2_gpio, 0)
                    self.off = False

        if not self.off:
            now = time.time()
            if not self.cruise_mode and now - self.last_throttle_input > 0.3 and self.throttle > neutral:
                self.throttle = max(self.throttle - step, neutral)

            if now - self.last_turn_input > 0.05 and self.turn != neutral:
                if self.turn > neutral:
                    self.turn = max(self.turn - turn_step, neutral)
                else:
                    self.turn = min(self.turn + turn_step, neutral)

            if self.cruise_mode:
                self.throttle = self.cruise_throttle

            pi.set_servo_pulsewidth(throttle_gpio, self.throttle)
            pi.set_servo_pulsewidth(turn_gpio, self.turn)

    def destroy_node(self):
        pi.set_servo_pulsewidth(throttle_gpio, 0)
        pi.set_servo_pulsewidth(turn_gpio, 0)
        pi.write(relay1_gpio, 1)
        pi.write(relay2_gpio, 1)
        pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
