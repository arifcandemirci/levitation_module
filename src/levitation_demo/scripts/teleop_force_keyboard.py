#!/usr/bin/env python3
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
Keyboard teleop for the levitation module
----------------------------------------
Move in plane:
    w : +X
    s : -X
    a : +Y
    d : -Y

Other keys:
    x : stop
    q : quit

The plugin interprets linear.x and linear.y in the range [-1, 1].
"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_force_keyboard')
        self.pub = self.create_publisher(Twist, '/levitation/cmd_vel', 10)

    def publish_cmd(self, x_value: float, y_value: float) -> None:
        msg = Twist()
        msg.linear.x = x_value
        msg.linear.y = y_value
        self.pub.publish(msg)


def get_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main() -> None:
    if not sys.stdin.isatty():
        print('teleop_force_keyboard.py requires an interactive terminal (TTY).')
        print('Run it in a regular terminal window, not a background/non-interactive shell.')
        return

    rclpy.init()
    node = TeleopNode()
    print(HELP)

    key_to_cmd = {
        'w': (0.2, 0.0),
        's': (-0.2, 0.0),
        'a': (0.0, 0.2),
        'd': (0.0, -0.2),
        'x': (0.0, 0.0),
    }

    try:
        while rclpy.ok():
            key = get_key()
            if key == 'q':
                node.publish_cmd(0.0, 0.0)
                break
            if key in key_to_cmd:
                x_value, y_value = key_to_cmd[key]
                node.publish_cmd(x_value, y_value)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
