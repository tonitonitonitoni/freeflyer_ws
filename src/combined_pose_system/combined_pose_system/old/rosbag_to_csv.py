#!/usr/bin/env python3

"""
Extract paired (x, y) trajectories from a ROS2 bag for frame alignment.

Reads:
  - /vision_pose/raw      (PoseWithCovarianceStamped)
  - /marvelmind/pose      (PoseWithCovarianceStamped)

Outputs:
  trajectory_pairs.csv with columns:
    x_v, y_v, x_mm, y_mm

Time matching policy:
  - Nearest-neighbor in time
  - Pairs accepted only if |t_v - t_mm| <= max_dt
  - Each Marvelmind sample is used at most once
"""

import sys
import csv
from bisect import bisect_left

import numpy as np

from rosbag2_py import SequentialReader  #type: ignore
from rosbag2_py import StorageOptions, ConverterOptions #type: ignore
from rclpy.serialization import deserialize_message #type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped #type: ignore


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_pose_topic(bag_path, topic_name):
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

    traj = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, PoseWithCovarianceStamped)

        t = stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        traj.append((t, x, y))

    if not traj:
        raise RuntimeError(f"No messages found on topic {topic_name}")

    traj.sort(key=lambda e: e[0])
    return traj


def time_match(traj_v, traj_m, max_dt):
    """
    Nearest-neighbor time matching.

    Vision samples are matched to the closest Marvelmind sample in time.
    Each Marvelmind sample may be used only once.
    """
    times_m = [e[0] for e in traj_m]
    used_m = [False] * len(traj_m)

    pairs = []
    dropped = 0

    for tv, xv, yv in traj_v:
        idx = bisect_left(times_m, tv)

        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(times_m):
            candidates.append(idx)

        best = None
        best_dt = None

        for j in candidates:
            if used_m[j]:
                continue
            dt = abs(times_m[j] - tv)
            if best is None or dt < best_dt:
                best = j
                best_dt = dt

        if best is None or best_dt > max_dt:
            dropped += 1
            continue

        used_m[best] = True
        _, xm, ym = traj_m[best]
        pairs.append((xv, yv, xm, ym))

    return pairs, dropped


def main():
    if len(sys.argv) < 2:
        print("Usage: rosbag_to_csv.py <bag_path> [max_dt]")
        sys.exit(1)

    bag_path = sys.argv[1]
    max_dt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05

    print(f"Reading bag: {bag_path}")
    print("Topics:")
    print("  vision:     vision/pose_raw")
    print("  marvelmind: /marvelmind/pose")
    print(f"Max time difference: {max_dt:.3f} s\n")

    vision_traj = read_pose_topic(bag_path, 'vision/pose_raw')
    mm_traj = read_pose_topic(bag_path, '/marvelmind/pose')

    pairs, dropped = time_match(vision_traj, mm_traj, max_dt)

    if len(pairs) < 5:
        raise RuntimeError("Not enough paired samples for alignment")

    out_path = "trajectory_pairs.csv"
    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x_v', 'y_v', 'x_mm', 'y_mm'])
        for row in pairs:
            writer.writerow(row)

    print("Pairing summary:")
    print(f"  Vision samples:     {len(vision_traj)}")
    print(f"  Marvelmind samples: {len(mm_traj)}")
    print(f"  Paired samples:     {len(pairs)}")
    print(f"  Dropped vision pts: {dropped}")
    print(f"\nWrote {out_path}\n")


if __name__ == '__main__':
    main()
