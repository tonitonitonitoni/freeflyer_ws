#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
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

        # ---------------- Command State ----------------
        self.cmd_px = 0.0
        self.cmd_nx = 0.0
        self.cmd_py = 0.0
        self.cmd_ny = 0.0
        self.cmd_rw = 0

        # ---------------- Subscriptions ----------------
        self.create_subscription(Float32, "/freeflyer/thrusters/px", self._cb_px, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/nx", self._cb_nx, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/py", self._cb_py, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/ny", self._cb_ny, 10)

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
    def _clamp(self, v):
        return max(0.0, min(1.0, v))

    def _cb_px(self, msg):
        self.cmd_px = self._clamp(msg.data)

    def _cb_nx(self, msg):
        self.cmd_nx = self._clamp(msg.data)

    def _cb_py(self, msg):
        self.cmd_py = self._clamp(msg.data)

    def _cb_ny(self, msg):
        self.cmd_ny = self._clamp(msg.data)

    def _cb_rw_cw(self, msg):
        self.cmd_rw = -1 if msg.data else 0

    def _cb_rw_ccw(self, msg):
        self.cmd_rw = 1 if msg.data else 0

    # -------------------------------------------------
    # Send Combined Command
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
            pwm_fwd = float(parts[3])
            pwm_rev = float(parts[4])
            pwm_lft = float(parts[5])
            pwm_rgt = float(parts[6])
            rw_state = float(parts[7])
            watchdog = float(parts[8])
        except ValueError:
            return

        self._actuators_msg.data = [pwm_fwd, pwm_rev, pwm_lft, pwm_rgt, rw_state]
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

    def send_command(self):

        # Map to Arduino ordering:
        # fwd, rev, lft, rgt
        fwd = self.cmd_px
        rev = self.cmd_nx
        lft = self.cmd_py
        rgt = self.cmd_ny
        rw = self.cmd_rw 

        cmd_line = f"$CMD,{fwd:.3f},{rev:.3f},{lft:.3f},{rgt:.3f},{rw}\n"

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
