#!/usr/bin/env python3

import json
import math
import numpy as np
import csv
# import sys


def umeyama_se2(src, dst):
    """
    Estimate SE(2) transform T such that:
        dst ≈ R * src + t

    src, dst: Nx2 arrays
    Returns: yaw (rad), tx, ty
    """

    assert src.shape == dst.shape
    n = src.shape[0]

    # Means
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)

    # Demean
    src_d = src - mu_src
    dst_d = dst - mu_dst

    # Covariance
    Sigma = (dst_d.T @ src_d) / n

    # SVD
    U, _, Vt = np.linalg.svd(Sigma)

    R = U @ Vt

    # Enforce proper rotation (no reflection)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    # Translation
    t = mu_dst - R @ mu_src

    # Extract yaw
    yaw = math.atan2(R[1, 0], R[0, 0])

    return yaw, t[0], t[1]


def load_csv(path):
    src = []
    dst = []

    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            src.append([float(row['x_v']), float(row['y_v'])])
            dst.append([float(row['x_mm']), float(row['y_mm'])])

    return np.array(src), np.array(dst)


def main():
    # if len(sys.argv) < 2:
    #     print("Usage: csv_transformation_map.py trajectory_pairs.csv")
    #     sys.exit(1)

    csv_path = "trajectory_pairs.csv" #sys.argv[1]

    src, dst = load_csv(csv_path)

    if src.shape[0] < 5:
        raise RuntimeError("Not enough data points for alignment")

    yaw, tx, ty = umeyama_se2(src, dst)

    print("\nEstimated vision_map → mm_map transform:")
    print("---------------------------------------")
    print(f"  yaw (deg): {math.degrees(yaw):.3f}")
    print(f"  tx (m):    {tx:.4f}")
    print(f"  ty (m):    {ty:.4f}")

    transform = {
        "source_frame": "vision_map",
        "target_frame": "mm_map",
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
        "tx": tx,
        "ty": ty
    }

    out_path = "vision_to_mm_transform.json"
    with open(out_path, 'w') as f:
        json.dump(transform, f, indent=2)

    print(f"\nSaved transform to {out_path}\n")


if __name__ == "__main__":
    main()
