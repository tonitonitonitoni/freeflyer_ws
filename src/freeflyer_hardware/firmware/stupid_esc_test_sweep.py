import serial
import time
import numpy as np

PORT = "/dev/ttyUSB0"   # change to your port
BAUD = 115200

THRUSTER = "fwd"  # fwd, rev, lft, rgt
STEP_COUNT = 15
HOLD_TIME = 2.5
SETTLE_TIME = 0.5
RESET_TIME = 1.0

command_indices = {
    "fwd": 0,
    "rev": 1,
    "lft": 2,
    "rgt": 3
}

# Map thruster name to index in command vector
thruster_index = command_indices[THRUSTER]

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)  # allow Arduino reset

print("Starting thrust sweep...")

def send_cmd(cmd_vec, rw=0):
    line = "$CMD,{:.3f},{:.3f},{:.3f},{:.3f},{}\n".format(
        cmd_vec[0], cmd_vec[1], cmd_vec[2], cmd_vec[3], rw
    )
    ser.write(line.encode())

# Generate sweep values from 0 to 1
u_values = np.linspace(0.0, 1.0, STEP_COUNT)

for u in u_values:
    print(f"Step command: {u:.3f}")

    # Reset to zero between steps
    send_cmd([0, 0, 0, 0])
    time.sleep(RESET_TIME)

    # Apply thrust
    cmd = [0, 0, 0, 0]
    cmd[thruster_index] = float(u)

    send_cmd(cmd)
    t_start = time.time()

    while time.time() - t_start < HOLD_TIME:
        # Keep publishing so transmitter watchdog doesn't fire
        send_cmd(cmd)
        time.sleep(0.05)

    print("  Done step")

# Final neutral
send_cmd([0, 0, 0, 0])
print("Sweep complete.")
