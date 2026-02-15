import numpy as np, cv2
from sensor_msgs.msg import Image, CompressedImage


def ros_image_to_numpy(msg: Image):
    """
    Convert sensor_msgs/Image to a NumPy array without cv_bridge.
    Assumes no row padding and standard encodings.
    """
    if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
        raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

    dtype = np.uint8
    img = np.frombuffer(msg.data, dtype=dtype)

    if msg.encoding in ('rgb8', 'bgr8'):
        expected_step = msg.width * 3
        if msg.step != expected_step:
            raise RuntimeError(
                f"Unexpected image stride: step={msg.step}, expected={expected_step}"
            )
        img = img.reshape((msg.height, msg.width, 3))
    else:  # mono8
        expected_step = msg.width
        if msg.step != expected_step:
            raise RuntimeError(
                f"Unexpected image stride: step={msg.step}, expected={expected_step}"
            )
        img = img.reshape((msg.height, msg.width))

    return img

def compressed_image_to_numpy(msg: CompressedImage, color=True):
    """
    Convert sensor_msgs/CompressedImage to a NumPy array without cv_bridge.

    Args:
        msg   : sensor_msgs/CompressedImage
        color : bool
            If True, return BGR image (H,W,3)
            If False, return grayscale image (H,W)

    Returns:
        img : np.ndarray
            Decoded image
    """
    if not msg.data:
        raise RuntimeError("CompressedImage contains no data")

    buf = np.frombuffer(msg.data, dtype=np.uint8)

    flag = cv2.IMREAD_COLOR if color else cv2.IMREAD_GRAYSCALE
    img = cv2.imdecode(buf, flag)

    if img is None:
        raise RuntimeError(
            f"cv2.imdecode failed (format={msg.format})"
        )

    return img