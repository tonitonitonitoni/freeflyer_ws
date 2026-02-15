import cv2
import numpy as np
import glob
import os

# ----------------------------
# USER INPUTS
# ----------------------------
CALIB_FILE = "camera_calibration_refined.npz"
TEST_IMAGE_DIR = "calib_images"   # can be ANY real images
SHOW_N = 5                        # number of images to visualize

# ChArUco board definition (only used if board appears in image)
CHARUCO_SQUARES_X = 7
CHARUCO_SQUARES_Y = 5
SQUARE_LENGTH = 0.035
MARKER_LENGTH = 0.026
ARUCO_DICT = cv2.aruco.DICT_4X4_100

# ----------------------------
# Load calibration
# ----------------------------
data = np.load(CALIB_FILE)
K = data["K"]
dist = data["dist"]

print("Loaded calibration:")
print("K:\n", K)
print("dist:", dist.ravel())

# ----------------------------
# Load images
# ----------------------------
images = sorted(
    glob.glob(os.path.join(TEST_IMAGE_DIR, "*.jpg")) +
    glob.glob(os.path.join(TEST_IMAGE_DIR, "*.png"))
)

if len(images) == 0:
    raise RuntimeError("No test images found")

# ----------------------------
# Setup ChArUco (optional reprojection test)
# ----------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(
    (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
    SQUARE_LENGTH,
    MARKER_LENGTH,
    aruco_dict,
)

aruco_detector = cv2.aruco.ArucoDetector(
    aruco_dict, cv2.aruco.DetectorParameters()
)

# ----------------------------
# Helper: visualize undistortion
# ----------------------------
def show_undistortion(img, K, dist):
    h, w = img.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1.0)
    undist = cv2.undistort(img, K, dist, None, newK)
    return undist

# ----------------------------
# Test loop
# ----------------------------
reproj_errors = []

for i, fname in enumerate(images[:SHOW_N]):
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    undist = show_undistortion(img, K, dist)

    # ---- Attempt ChArUco reprojection ----
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    reproj_err = None

    if ids is not None and len(ids) >= 4:
        ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )

        if ret and ch_ids is not None and len(ch_ids) >= 6:
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                ch_corners, ch_ids, board, K, dist, None, None
            )

            if ok:
                imgpts, _ = cv2.projectPoints(
                    board.chessboardCorners[ch_ids.flatten()],
                    rvec,
                    tvec,
                    K,
                    dist,
                )

                imgpts = imgpts.reshape(-1, 2)
                obs = ch_corners.reshape(-1, 2)

                reproj_err = np.mean(
                    np.linalg.norm(obs - imgpts, axis=1)
                )
                reproj_errors.append(reproj_err)

                # draw reprojection
                vis = img.copy()
                for p in imgpts:
                    cv2.circle(vis, tuple(p.astype(int)), 3, (0, 0, 255), -1)
            else:
                vis = img.copy()
        else:
            vis = img.copy()
    else:
        vis = img.copy()

    # ---- Display ----
    stacked = np.hstack([img, undist, vis])
    cv2.imshow(f"Original | Undistorted | Reprojection [{i}]", stacked)
    print(f"[{i}] reprojection error: {reproj_err}")
    cv2.waitKey(0)

cv2.destroyAllWindows()

# ----------------------------
# Summary
# ----------------------------
if reproj_errors:
    print("\n=== Reprojection error summary ===")
    print("mean:", np.mean(reproj_errors))
    print("std :", np.std(reproj_errors))
else:
    print("\nNo ChArUco boards detected in test images")
