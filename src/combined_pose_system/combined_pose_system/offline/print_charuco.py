import cv2
import numpy as np

# ----------------------------
# User parameters
# ----------------------------
CHARUCO_SQUARES_X = 7
CHARUCO_SQUARES_Y = 5
SQUARE_LENGTH = 0.035   # meters - MEASURE THIS CAREFULLY!
MARKER_LENGTH = 0.026   # meters
ARUCO_DICT = cv2.aruco.DICT_4X4_100
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(
    (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
    SQUARE_LENGTH,
    MARKER_LENGTH,
    aruco_dict,
)

img = board.generateImage((2000, 1400))
cv2.imwrite("charuco_board.png", img)
