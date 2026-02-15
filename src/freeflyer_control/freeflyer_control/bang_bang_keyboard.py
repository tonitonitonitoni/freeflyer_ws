#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys, termios, tty

class KeyboardBangBang(Node):
    def __init__(self):
        super().__init__("keyboard_bangbang")
        self.declare_parameter("thrust_level", 1.0)
        self.thrust_level = float(self.get_parameter("thrust_level").value)
        self.thrust_level = max(0.0, min(1.0, self.thrust_level))

        # Thruster publishers
        self.thrusters = {
            'px': self.create_publisher(Float32, '/freeflyer/thrusters/px', 10),
            'nx': self.create_publisher(Float32, '/freeflyer/thrusters/nx', 10),
            'py': self.create_publisher(Float32, '/freeflyer/thrusters/py', 10),
            'ny': self.create_publisher(Float32, '/freeflyer/thrusters/ny', 10),
        }

        self.get_logger().info(
            "Keyboard thruster control (Float32):\n"
            "  W/S : +X / -X\n"
            "  A/D : +Y / -Y\n"
            "  Q/E : (yaw torque – reserved)\n"
            "  SPACE: all thrusters OFF\n"
            f"  thrust_level: {self.thrust_level:.2f}\n"
        )

    def run(self):
        while rclpy.ok():
            key = getch()
            self.handle_key(key)

    def handle_key(self, key):
        # Default: all OFF (float command in [0,1])
        cmd = {
            'px': 0.0,
            'nx': 0.0,
            'py': 0.0,
            'ny': 0.0,
        }

        if key == 'w': cmd['px'] = self.thrust_level
        elif key == 's': cmd['nx'] = self.thrust_level
        elif key == 'a': cmd['py'] = self.thrust_level
        elif key == 'd': cmd['ny'] = self.thrust_level
        elif key == ' ': pass   # explicit all-off
        elif key in ['q', 'e']:
            # reserved for reaction wheel / yaw thrusters
            self.get_logger().info("Yaw torque key pressed (not implemented yet)")
            return
        else:
            return  # ignore unknown keys

        self.publish_cmd(cmd)

    def publish_cmd(self, cmd):
        for name, pub in self.thrusters.items():
            msg = Float32()
            msg.data = float(cmd[name])
            pub.publish(msg)

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = KeyboardBangBang()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
