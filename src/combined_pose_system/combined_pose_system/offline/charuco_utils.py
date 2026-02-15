from types import SimpleNamespace
import cv2, numpy as np
CHARUCO_PARAMS = {
    'squares_x':        7,
    'squares_y':        5,
    'square_length':    0.035,
    'marker_length':    0.026,
    "image_dir":        "charuco_calib_images"
}
params = SimpleNamespace(**CHARUCO_PARAMS)

def show_undistort(title, img, K, dist):
    undist = cv2.undistort(img, K, dist)
    stacked = np.hstack([img, undist])
    cv2.imshow(title, stacked)
    cv2.waitKey(0)
# ----------------------------
# Create board + detector
# ----------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_100
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(
    (params.squares_x, params.squares_y),
    params.square_length,
    params.marker_length,
    aruco_dict,
)
detector_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
