import cv2
import numpy as np
import glob
import os
from combined_pose_system.offline.charuco_utils import params, aruco_detector, board

def detect_charuco_corners(img, aruco_detector, VISUALIZE=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if img_size is None:
        img_size = (gray.shape[1], gray.shape[0])  # (width, height)

    # Detect ArUco markers
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    if ids is None or len(ids) < 4:
        return

    # Interpolate ChArUco corners
    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=gray,
        board=board,
    )

    if not ret or charuco_ids is None or len(charuco_ids) < 6:
        return

    if VISUALIZE:
        vis = img.copy()
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
        cv2.imshow("ChArUco Detection", vis)
        cv2.waitKey(150)
    return charuco_corners, charuco_ids

# ----------------------------
# Load images
# ----------------------------
images = sorted(
    glob.glob(os.path.join(params.image_dir, "*.jpg")) +
    glob.glob(os.path.join(params.image_dir, "*.png"))
)

if len(images) == 0:
    raise RuntimeError("No calibration images found")

print(f"[CHARUCO] Found {len(images)} images")

# ----------------------------
# Collect detections
# ----------------------------
all_charuco_corners = []
all_charuco_ids = []
img_size = None

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"[WARN] Failed to load {fname}")
        continue
    
    charuco_corners, charuco_ids = detect_charuco_corners(img)
    all_charuco_corners.append(charuco_corners)
    all_charuco_ids.append(charuco_ids)


cv2.destroyAllWindows()

if len(all_charuco_corners) < 5:
    raise RuntimeError(
        f"Not enough valid ChArUco views ({len(all_charuco_corners)})"
    )

# ----------------------------
# Calibrate camera
# ----------------------------
ret, K, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    charucoCorners=all_charuco_corners,
    charucoIds=all_charuco_ids,
    board=board,
    imageSize=img_size, 
    cameraMatrix=None, 
    distCoeffs=None, 
)

print("\n=== ChArUco Calibration Results ===")
print("Reprojection error:", ret)
print("\nCamera matrix K:\n", K)
print("\nDistortion coefficients:\n", dist.ravel())

# ----------------------------
# Save results
# ----------------------------
np.savez(
    "camera_calibration_charuco.npz",
    K=K,
    dist=dist,
    reprojection_error=ret,
)

print("\nSaved to camera_calibration_charuco.npz")
