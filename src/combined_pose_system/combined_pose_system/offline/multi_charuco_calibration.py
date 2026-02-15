import cv2
import numpy as np
import glob
import os
from charuco_utils import params, show_undistort, aruco_detector, board


INITIAL_CALIB = "camera_calibration_charuco.npz"  # or None
VISUALIZE = True

# ----------------------------
# Load images
# ----------------------------
images = sorted(
    glob.glob(os.path.join(params.image_dir, "*.jpg")) +
    glob.glob(os.path.join(params.image_dir, "*.png"))
)

if len(images) == 0:
    raise RuntimeError("No calibration images found")

# ----------------------------
# Detect ChArUco corners
# ----------------------------
all_charuco_corners = []
all_charuco_ids = []
img_size = None
example_imgs = []

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img_size is None:
        img_size = (gray.shape[1], gray.shape[0])

    corners, ids, _ = aruco_detector.detectMarkers(gray)
    if ids is None or len(ids) < 4:
        continue

    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )

    if not ret or charuco_ids is None or len(charuco_ids) < 6:
        continue

    all_charuco_corners.append(charuco_corners)
    all_charuco_ids.append(charuco_ids)
    example_imgs.append(img)

if len(all_charuco_corners) < 5:
    raise RuntimeError("Not enough valid ChArUco detections")

# ----------------------------
# Stage 0 — Initial calibration
# ----------------------------
print("\n[STAGE 0] Initial calibration")

K0 = None
dist0 = None

if INITIAL_CALIB and os.path.exists(INITIAL_CALIB):
    data = np.load(INITIAL_CALIB)
    K0 = data["K"]
    dist0 = data["dist"]
    print("Loaded initial calibration")
else:
    print("Computing initial calibration from scratch")
    ret0, K0, dist0, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners,
        all_charuco_ids,
        board,
        img_size,
        None,
        None,
    )
print("Reprojection error:", ret0)
print("K0:\n", K0)
print("dist0:", dist0.ravel())


# ----------------------------
# Stage 1 — Refinement (fix principal point + fx/fy)
# ----------------------------
print("\n[STAGE 1] Refinement")

flags = (
    cv2.CALIB_USE_INTRINSIC_GUESS
)

ret1, K1, dist1, _, _ = cv2.aruco.calibrateCameraCharuco(
    all_charuco_corners,
    all_charuco_ids,
    board,
    img_size,
    K0.copy(),
    dist0.copy(),
    flags=flags,
)

print("Reprojection error:", ret1)
print("K1:\n", K1)
print("dist1:", dist1.ravel())

# ----------------------------
# Stage 2 — Refine with fixed aspect ratio (optional)
# ----------------------------
print("\n[STAGE 2] Aspect-ratio constrained refinement")

K2 = K1.copy()
K2[1, 1] = K2[0, 0]  # enforce fy = fx

flags = (
    cv2.CALIB_USE_INTRINSIC_GUESS |
    cv2.CALIB_FIX_ASPECT_RATIO
)

ret2, K2, dist2, _, _ = cv2.aruco.calibrateCameraCharuco(
    all_charuco_corners,
    all_charuco_ids,
    board,
    img_size,
    K2,
    dist1.copy(),
    flags=flags,
)

print("Reprojection error:", ret2)
print("K2:\n", K2)
print("dist2:", dist2.ravel())

# ----------------------------
# Stage 3 — Visual inspection
# ----------------------------
print("\n[STAGE 3] Visual undistortion check")

for i, img in enumerate(example_imgs[:3]):
    show_undistort(f"Stage0 vs Stage1 vs Stage2 [{i}]", img, K2, dist2)

# ----------------------------
# Save final calibration
# ----------------------------
np.savez(
    "camera_calibration_refined.npz",
    K=K2,
    dist=dist2,
    reprojection_error=ret2,
)

print("\nSaved refined calibration to camera_calibration_refined.npz")
cv2.destroyAllWindows()
