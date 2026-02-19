#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def get_key(timeout=0.1):
    """Non-blocking key reader."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None


class FreeflyerKeyboard(Node):
    def __init__(self):
        super().__init__("freeflyer_keyboard")

        # Thruster publishers (bang-bang)
        self.thrusters = {
            "px": self.create_publisher(Bool, "/freeflyer/thrusters/px", 10),
            "nx": self.create_publisher(Bool, "/freeflyer/thrusters/nx", 10),
            "py": self.create_publisher(Bool, "/freeflyer/thrusters/py", 10),
            "ny": self.create_publisher(Bool, "/freeflyer/thrusters/ny", 10),
        }

        # Continuous reaction wheel
        self.rw_pub = self.create_publisher(Float32,
                                            "/freeflyer/reaction_wheel/cmd",
                                            10)

        self.rw_cmd = 0.0
        self.timer = self.create_timer(0.05, self.loop)

        print("""
Controls:
  w/s  : +X / -X thrust
  a/d  : +Y / -Y thrust
  q/e  : reaction wheel CCW / CW
  space: all off
  x    : exit
        """)

    def publish_thruster(self, name, state):
        msg = Bool()
        msg.data = state
        self.thrusters[name].publish(msg)

    def all_thrusters_off(self):
        for name in self.thrusters:
            self.publish_thruster(name, False)

    def loop(self):
        key = get_key()

        # Default: thrusters off unless key held
        self.all_thrusters_off()

        if key is None:
            # Still publish RW command (continuous)
            msg = Float32()
            msg.data = self.rw_cmd
            self.rw_pub.publish(msg)
            return

        if key == 'w':
            self.publish_thruster("px", True)
        elif key == 's':
            self.publish_thruster("nx", True)
        elif key == 'a':
            self.publish_thruster("py", True)
        elif key == 'd':
            self.publish_thruster("ny", True)

        elif key == 'q':
            self.rw_cmd += 0.1
            self.rw_cmd = min(self.rw_cmd, 1.0)
        elif key == 'e':
            self.rw_cmd -= 0.1
            self.rw_cmd = max(self.rw_cmd, -1.0)

        elif key == ' ':
            self.rw_cmd = 0.0
            self.all_thrusters_off()

        elif key == 'x':
            print("Exiting teleop.")
            rclpy.shutdown()
            return

        # Publish reaction wheel command every tick
        msg = Float32()
        msg.data = self.rw_cmd
        self.rw_pub.publish(msg)


def main():
    rclpy.init()

    # Setup terminal raw mode
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    node = FreeflyerKeyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
