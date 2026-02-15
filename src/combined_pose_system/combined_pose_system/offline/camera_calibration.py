import cv2
import numpy as np
import glob
import os
# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
# ----------------------------
# User parameters
# ----------------------------
CHECKERBOARD = (9, 6)     # (cols, rows) inner corners
SQUARE_SIZE = 0.0245     # meters
IMAGE_DIR = "calib_images"   # directory with checkerboard images
VISUALIZE = True

# ----------------------------
# Prepare object points
# ----------------------------
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0],
                       0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D points in checkerboard frame
imgpoints = []  # 2D points in image

# ----------------------------
# Load images
# ----------------------------
images = sorted(
    glob.glob(os.path.join(IMAGE_DIR, "*.jpg")) +
    glob.glob(os.path.join(IMAGE_DIR, "*.png"))
)

if len(images) == 0:
    raise RuntimeError("No calibration images found")

print(f"[CALIB] Found {len(images)} images")

# ----------------------------
# Detect corners
# ----------------------------
for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"[WARN] Failed to load {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
              cv2.CALIB_CB_NORMALIZE_IMAGE +
              cv2.CALIB_CB_FAST_CHECK
    )

    if not found:
        print(f"[WARN] Chessboard not found in {fname}")
        continue

    # Refine corner locations
    corners = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=(
            cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER,
            30,
            1e-6
        )
    )

    objpoints.append(objp)
    imgpoints.append(corners)

    if VISUALIZE:
        vis = img.copy()
        cv2.drawChessboardCorners(vis, CHECKERBOARD, corners, found)
        cv2.imshow("Calibration", vis)
        cv2.waitKey(150)

cv2.destroyAllWindows()

if len(objpoints) < 8:
    raise RuntimeError(f"Not enough valid calibration views: {len(objpoints)}")

# Ensure correct dtypes (important for Python bindings)
objpoints = [np.asarray(pts, dtype=np.float32) for pts in objpoints]
imgpoints = [np.asarray(pts, dtype=np.float32) for pts in imgpoints]

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None) 

print("Reprojection error:", ret)
print("K:\n", K)
print("dist:\n", dist.ravel())


# ----------------------------
# Save results
# ----------------------------
np.savez(
    "camera_calib.npz",
    K=K,
    dist=dist,
    reprojection_error=ret,
)

print("\nSaved to camera_calib.npz")
