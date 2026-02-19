#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import serial


class SerialBridge(Node):

    def __init__(self):
        super().__init__('ff_serial_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_rate', 50.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        send_rate = self.get_parameter('send_rate').value

        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.get_logger().info(f"Opened serial {port} @ {baudrate}")

        # ---------------- Command State (Booleans) ----------------
        self.cmd_px = False
        self.cmd_nx = False
        self.cmd_py = False
        self.cmd_ny = False
        self.cmd_rw = 0  # -1, 0, 1

        # ---------------- Subscriptions ----------------
        self.create_subscription(Bool, "/freeflyer/thrusters/px", self._cb_px, 10)
        self.create_subscription(Bool, "/freeflyer/thrusters/nx", self._cb_nx, 10)
        self.create_subscription(Bool, "/freeflyer/thrusters/py", self._cb_py, 10)
        self.create_subscription(Bool, "/freeflyer/thrusters/ny", self._cb_ny, 10)

        self.create_subscription(Bool, "/freeflyer/reaction_wheel/cw", self._cb_rw_cw, 10)
        self.create_subscription(Bool, "/freeflyer/reaction_wheel/ccw", self._cb_rw_ccw, 10)

        # ---------------- Telemetry Publishers ----------------
        self.actuators_pub = self.create_publisher(
            Float32MultiArray, "/freeflyer/telemetry/actuators", 10
        )
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray, "/freeflyer/telemetry/diagnostics", 10
        )
        self._actuators_msg = Float32MultiArray()
        self._diagnostics_msg = Float32MultiArray()
        self._rx_buf = bytearray()

        # ---------------- Send Timer ----------------
        self.timer = self.create_timer(1.0 / send_rate, self.send_command)

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------

    def _cb_px(self, msg):
        self.cmd_px = msg.data

    def _cb_nx(self, msg):
        self.cmd_nx = msg.data

    def _cb_py(self, msg):
        self.cmd_py = msg.data

    def _cb_ny(self, msg):
        self.cmd_ny = msg.data

    def _cb_rw_cw(self, msg):
        if msg.data:
            self.cmd_rw = -1
        elif self.cmd_rw == -1:
            self.cmd_rw = 0

    def _cb_rw_ccw(self, msg):
        if msg.data:
            self.cmd_rw = 1
        elif self.cmd_rw == 1:
            self.cmd_rw = 0

    # -------------------------------------------------
    # Telemetry Parsing
    # -------------------------------------------------

    def _parse_and_publish_tel(self, line: str):
        if not line.startswith("$TEL,"):
            return

        parts = line.strip().split(",")
        if len(parts) < 9:
            return

        try:
            packet = float(parts[1])
            last_rx = float(parts[2])

            # These are booleans coming back as 0/1
            fwd = float(parts[3])
            rev = float(parts[4])
            lft = float(parts[5])
            rgt = float(parts[6])
            rw_state = float(parts[7])
            watchdog = float(parts[8])

        except ValueError:
            return

        self._actuators_msg.data = [fwd, rev, lft, rgt, rw_state]
        self._diagnostics_msg.data = [packet, last_rx, watchdog]

        self.actuators_pub.publish(self._actuators_msg)
        self.diagnostics_pub.publish(self._diagnostics_msg)

    def _read_telemetry(self):
        waiting = self.ser.in_waiting
        if waiting <= 0:
            return

        self._rx_buf.extend(self.ser.read(waiting))

        while True:
            nl = self._rx_buf.find(b"\n")
            if nl < 0:
                break

            line = self._rx_buf[:nl].decode("ascii", errors="ignore")
            del self._rx_buf[:nl + 1]
            self._parse_and_publish_tel(line)

    # -------------------------------------------------
    # Send Combined Command
    # -------------------------------------------------

    def send_command(self):

        # Convert booleans to 0/1 integers
        fwd = 1 if self.cmd_px else 0
        rev = 1 if self.cmd_nx else 0
        lft = 1 if self.cmd_py else 0
        rgt = 1 if self.cmd_ny else 0
        rw = self.cmd_rw

        cmd_line = f"$CMD,{fwd},{rev},{lft},{rgt},{rw}\n"

        try:
            self.ser.write(cmd_line.encode())
            self._read_telemetry()
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
