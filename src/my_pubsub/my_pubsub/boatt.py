import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import curses
import time

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher_ = self.create_publisher(JointState, 'cmd_motor', 10)
        self.msg = JointState()
        self.msg.name = ['']
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_key = ''

    def timer_callback(self):
        if self.last_key:
            self.msg.name[0] = self.last_key
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.msg)
            self.get_logger().info(f'Sent key: {self.last_key}')
            self.last_key = ''

def curses_loop(stdscr, node):
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Use WASD or arrow keys | B: Brake | R: Relay | Q: Reset | C: Cruise | ESC: Quit")

    key_map = {
        curses.KEY_UP: 'w',
        curses.KEY_DOWN: 's',
        curses.KEY_LEFT: 'a',
        curses.KEY_RIGHT: 'd',
        ord('w'): 'w',
        ord('s'): 's',
        ord('a'): 'a',
        ord('d'): 'd',
        ord('b'): 'b',
        ord(' '): 'b',
        ord('r'): 'r',
        ord('q'): 'q',
        ord('c'): 'c'
    }

    while rclpy.ok():
        key = stdscr.getch()
        if key == 27:  # ESC
            break
        if key in key_map:
            node.last_key = key_map[key]

        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.01)

def main():
    rclpy.init()
    node = MotorCommandPublisher()
    try:
        curses.wrapper(curses_loop, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
