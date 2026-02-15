#!/usr/bin/env python3

"""
Estimate planar pose covariance from a time series.

Input options:
  1) CSV file with columns: t, x, y, yaw
  2) ROS2 bag containing PoseWithCovarianceStamped messages

This script is intended to be run on a STATIONARY segment
to estimate measurement noise for EKF tuning.

Examples:
  CSV:
    python estimate_pose_covariance.py csv pose.csv

  ROS bag:
    python estimate_pose_covariance.py bag my_bag /vision_pose/mm_map
"""

import csv
import math
import sys
import numpy as np

from combined_pose_system.utils.utils_ros import wrap_angle, yaw_from_quat

from rosbag2_py import SequentialReader #type: ignore
from rosbag2_py import StorageOptions, ConverterOptions #type: ignore
from rclpy.serialization import deserialize_message #type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped #type: ignore

def unwrap_yaw(yaws):
    """Unwrap yaw to a continuous signal."""
    unwrapped = [yaws[0]]
    for y in yaws[1:]:
        dy = wrap_angle(y - unwrapped[-1])
        unwrapped.append(unwrapped[-1] + dy)
    return np.array(unwrapped)

def load_csv(path):
    t, x, y, yaw = [], [], [], []

    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            yaw.append(float(row['yaw']))

    return np.array(t), np.array(x), np.array(y), np.array(yaw)


def load_rosbag(bag_path, topic_name):
    reader = SequentialReader()
    storage_options = StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    t, x, y, yaw = [], [], [], []

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, PoseWithCovarianceStamped)

        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t.append(sec)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        yaw.append(yaw_from_quat(q))

    return np.array(t), np.array(x), np.array(y), np.array(yaw)


def main():
    if len(sys.argv) < 3:
        print(
            "Usage:\n"
            "  CSV: estimate_pose_covariance.py csv pose.csv\n"
            "  BAG: estimate_pose_covariance.py bag <bag_path> <topic>"
        )
        sys.exit(1)

    mode = sys.argv[1]

    if mode == 'csv':
        csv_path = sys.argv[2]
        t, x, y, yaw = load_csv(csv_path)

    elif mode == 'bag':
        if len(sys.argv) < 4:
            raise RuntimeError("Bag mode requires <bag_path> and <topic>")
        bag_path = sys.argv[2]
        topic = sys.argv[3]
        t, x, y, yaw = load_rosbag(bag_path, topic)

    else:
        raise RuntimeError("Mode must be 'csv' or 'bag'")

    if len(t) < 10:
        raise RuntimeError("Not enough samples to estimate covariance")

    # Remove mean (stationary assumption)
    x_d = x - np.mean(x)
    y_d = y - np.mean(y)

    yaw_unwrapped = unwrap_yaw(yaw)
    yaw_d = yaw_unwrapped - np.mean(yaw_unwrapped)

    cov = np.cov(np.vstack([x_d, y_d, yaw_d]), bias=True)

    print("\nEstimated planar pose covariance:")
    print("--------------------------------")
    print(f"Var(x):   {cov[0,0]:.6e}  (std = {math.sqrt(cov[0,0]):.4e} m)")
    print(f"Var(y):   {cov[1,1]:.6e}  (std = {math.sqrt(cov[1,1]):.4e} m)")
    print(f"Var(yaw): {cov[2,2]:.6e}  (std = {math.sqrt(cov[2,2]):.4e} rad)")

    print("\nFull covariance matrix:")
    print(cov)


if __name__ == "__main__":
    main()