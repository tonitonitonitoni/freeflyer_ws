import numpy as np
import matplotlib.pyplot as plt
import cv2
from sklearn.cluster import DBSCAN

def find_centroids(img, visualize=False, 
                   lower_green=np.array([40, 100, 40]), 
                   upper_green=np.array([80, 255, 255]), 
                   lower_blue = np.array([100, 100, 40]), 
                   upper_blue = np.array([140, 255, 255]), 
                   vis_masks=False, 
                   value=True,
                   area_threshold=10, 
                   dist_threshold=5, 
                   blue_threshold=120,
                   green_threshold=120, 
                   write=False, fname=None):

    # convert image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # Extract blue channel
    blue = img[:, :, 0]
    green = img[:,:,1]
    _, blue_blue_mask = cv2.threshold(blue, blue_threshold, 255, cv2.THRESH_BINARY)
    _, green_green_mask = cv2.threshold(green, green_threshold, 255, cv2.THRESH_BINARY)
    if value:
        # combine blue masks for the ULTIMATE BLUE
        blue_mask = cv2.bitwise_and(blue_mask, blue_blue_mask)
        green_mask = cv2.bitwise_and(green_mask, green_green_mask)

    if vis_masks:
        plt.imshow(green_mask, cmap='Greys')
        plt.title(f"Green")
        plt.show()
        plt.imshow(blue_mask, cmap='Greys')
        plt.title(f"Blue")
        plt.show()
    led_centroids = []
    i = 0
    for mask in [green_mask, blue_mask]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Get LED centroids
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > area_threshold:  # Area threshold
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    new_centroid = (cx, cy)
                    check_dist = [distance(new_centroid, c) for c in led_centroids]
                    if len(check_dist) > 0:
                        if min(check_dist) > dist_threshold:  # Distance threshold
                            led_centroids.append((cx, cy, i))
                    else:
                        led_centroids.append((cx, cy, i))
                    # led_centroids.append((cx, cy, i))
        i += 1
    led_centroids = np.array(led_centroids)

    return led_centroids

# Patterning
def find_patterns(points, eps=500):
    centroids = points[:, 0:2]
    clustering = DBSCAN(eps=eps, min_samples=3).fit(centroids)
    labels = clustering.labels_
    # Extract clusters
    num = len(set(labels)) - (1 if -1 in labels else 0)
    pats = []
    for j in range(num):
        pattern = points[labels == j]
        if len(pattern) == 3:
            pats.append(pattern)
    return pats, num


# Identify pattern
def is_collinear(points, tol=2.5):
    """Use PCA to determine if 4 points lie on a line."""
    pts = np.array(points)
    mean = np.mean(pts, axis=0)
    centered = pts - mean
    _, s, _ = np.linalg.svd(centered)
    return s[1] < tol  # second singular value close to 0

# Useful math functions
def make_vector(p1, p2):
    p1_coords = np.array(p1[0:2])
    p2_coords = np.array(p2[0:2])
    return p2_coords - p1_coords

def distance(p1, p2):
    return np.linalg.norm(make_vector(p1, p2))

def angle_between(v1, v2):
    cos_angle = np.dot(v1, v2)/(np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.arccos(abs(cos_angle))
    return int(np.round(np.rad2deg(angle), 0))

# Identify pattern
def angles_in_triangle(A, B, C):
    #Sort triangle vertices by ascending angle
    side_AB = make_vector(B, A)
    side_AC = make_vector(C, A)
    side_BC = make_vector(C, B)
    angle_B = angle_between(side_AB, side_BC)
    angle_A = angle_between(side_AB, side_AC)
    angle_C = angle_between(side_BC, side_AC)
    point_list = [[angle_A, A], [angle_B, B], [angle_C, C]]
    sorted_point_list = sorted(point_list)
    coords = [point[1:] for point in sorted_point_list]
    return coords

def arrange_by_distance(A, B, C):
    AB = distance(A, B)
    AC = distance(A, C)
    BC = distance(B, C)
    # lines arranged  *  *     *
    #A B C; A C B ; B A C; B C A; C A B ; C B A
    if AB < BC < AC:
        return A, B, C
    if AC < BC < AB:
        return A, C, B
    if AB < AC < BC:
        return B, A, C
    if BC < AC  < AB:
        return B, C, A
    if AC < AB  < BC:
        return C, A, B
    if BC < AB < AC:
        return C, B, A

def relative_transformation_with_scale(A, B):
    """
    An AI wrote this.

    Calculates the best-fit similarity transform (rotation, translation, scale)
    that maps points A to points B using a scaled version of the Kabsch algorithm.

    Args:
        A: NxD numpy array of source points
        B: NxD numpy array of destination points

    Returns:
        R: DxD rotation matrix
        t: Dx1 translation vector
        s: scalar scale factor
    """
    assert A.shape == B.shape, "Point sets must have the same shape"

    # Compute centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Center the points
    AA = A - centroid_A
    BB = B - centroid_B

    # Compute covariance matrix
    H = AA.T @ BB

    # SVD
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Correct reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute scale
    var_A = np.sum(AA ** 2)
    s = np.sum(S) / var_A

    # Compute translation
    t = centroid_B - s * R @ centroid_A

    return R, t, s

def apply_similarity_transform(A, R, t, s):

    # A = input point set
    # R, t, s are calculated by the relative_transform_with_scale function

    return (s * (R @ A.T)).T + t


def identify_cluster(pat):
    pat_cols = [c for x, y, c in pat]
    colors = sorted(pat_cols)
    if is_collinear(pat):
        if colors == [0, 0, 0]: # Green, Green, Green
            cluster_index = 3
        elif colors == [0, 0, 1]:  # Green, Green, Blue
            cluster_index = 1
        elif colors == [0, 1, 1]:  # Green, Blue, Blue
            cluster_index = 7
        elif colors == [1, 1, 1]:  # Blue, Blue, Blue
            cluster_index = 4
        else:
            print("Can't identify cluster")
            cluster_index = -1
    else:
        if colors == [0, 0, 0]: # Green, Green, Green
            cluster_index = 6
        elif colors == [0, 0, 1]:  # Green, Green, Blue
            cluster_index = 0
        elif colors == [0, 1, 1]:  # Green, Blue, Blue
            cluster_index = 5
        elif colors == [1, 1, 1]:  # Blue, Blue, Blue
            cluster_index = 2
        else:
            print("Can't identify cluster")
            cluster_index = -1

    return cluster_index


def sort_cluster(pat):
    if is_collinear(pat):
        sorted_points = arrange_by_distance(*pat)
    else:
        sorted_points = angles_in_triangle(*pat)
    return sorted_points


def points_to_array(point_list):
    point_array = []
    for points in point_list:
        for point in points:
            point_array.append(point)
    unnested_list = []
    for point in point_array:
        if len(point) < 2:
            unnested_list.append(point[0])
        else:
            unnested_list.append(point)
    return np.array(unnested_list)

def pixels_to_cm(pixel_position, pixel_error, scale_factor=None):
    # Convert pixel coordinates to real-world centimetres using cached scale factor
    if scale_factor is None: scale_factor = 0.11641121527233193
    error_cm = pixel_error * scale_factor
    world_position = pixel_position * scale_factor
    return world_position, error_cm

def compute_scale(sample_obs):
    # Compute pixel->cm scale from the collinear blue cluster (cluster 4)
    sample_centroids = find_centroids(sample_obs, area_threshold=10, dist_threshold=5)
    patterns, _ = find_patterns(sample_centroids)

    # Search all patterns
    for pat in patterns:
        if identify_cluster(pat) == 4:  # Blue, Blue, Blue (collinear)
            pts = pat[:, 0:2].astype(float)
            # Find endpoints (max pairwise distance) and middle point
            dists = np.array([
                [0.0, np.linalg.norm(pts[0] - pts[1]), np.linalg.norm(pts[0] - pts[2])],
                [np.linalg.norm(pts[1] - pts[0]), 0.0, np.linalg.norm(pts[1] - pts[2])],
                [np.linalg.norm(pts[2] - pts[0]), np.linalg.norm(pts[2] - pts[1]), 0.0],
            ])
            i, j = (int(v) for v in np.unravel_index(np.argmax(dists), dists.shape))
            k = ({0, 1, 2} - {i, j}).pop()
            d1 = np.linalg.norm(pts[i] - pts[k])
            d2 = np.linalg.norm(pts[j] - pts[k])
            # Known real distances: 14" and 10" between middle and endpoints
            cm1, cm2 = 14 * 2.54, 10 * 2.54
            # Weighted average for robustness
            cm_per_px = (cm1 + cm2) / (d1 + d2) if (d1 + d2) > 0 else 1.0
            return cm_per_px

    # No matching cluster found
    return None

def estimate_planar_pose(sample_obs, cluster_locs):
    """
    Returns:
        x_cm, y_cm, yaw_rad, error_cm
        or None if pose could not be computed
    """
    sample_centroids = find_centroids(sample_obs, area_threshold=10, dist_threshold=5)
    patterns, _ = find_patterns(sample_centroids)

    observed_point_list = []
    true_point_list = []

    for pat in patterns:
        observed_points = pat[:, 0:2].tolist()
        sorted_observed_points = sort_cluster(observed_points)

        cluster_index = identify_cluster(pat)
        if cluster_index >= 0:
            observed_point_list.append(sorted_observed_points)
            true_points = cluster_locs[str(cluster_index)]
            sorted_true_points = sort_cluster(true_points)
            true_point_list.append(sorted_true_points)

    if len(observed_point_list) == 0:
        return None

    opl = points_to_array(observed_point_list)
    tpl = points_to_array(true_point_list)

    R, t, s = relative_transformation_with_scale(opl, tpl)

    sample_centre = np.array([
        sample_obs.shape[1] / 2,
        sample_obs.shape[0] / 2
    ])

    position_px = apply_similarity_transform(sample_centre, R, t, s)
    position_cm, error_cm = pixels_to_cm(position_px, np.zeros_like(position_px))

    yaw_rad = np.arctan2(R[1, 0], R[0, 0])

    return position_cm[0], position_cm[1], yaw_rad, np.linalg.norm(error_cm)

def estimate_planar_yaw(sample_obs, cluster_locs):
    """
    Returns:
        yaw_rad, yaw_err_rad
        or None if yaw could not be computed
    """

    sample_centroids = find_centroids(sample_obs, area_threshold=10, dist_threshold=5)
    patterns, _ = find_patterns(sample_centroids)

    observed_point_list = []
    true_point_list = []

    for pat in patterns:
        observed_points = pat[:, 0:2].tolist()
        sorted_observed_points = sort_cluster(observed_points)

        cluster_index = identify_cluster(pat)
        if cluster_index >= 0:
            observed_point_list.append(sorted_observed_points)
            true_points = cluster_locs[str(cluster_index)]
            sorted_true_points = sort_cluster(true_points)
            true_point_list.append(sorted_true_points)

    if len(observed_point_list) == 0:
        return None

    opl = points_to_array(observed_point_list)
    tpl = points_to_array(true_point_list)

    # Similarity transform
    R, t, s = relative_transformation_with_scale(opl, tpl)

    # --- Yaw ---
    yaw_rad = np.arctan2(R[1, 0], R[0, 0])

    # --- Angular error estimate ---
    AA = opl - opl.mean(axis=0)
    BB = tpl - tpl.mean(axis=0)

    AA_rot = (R @ AA.T).T
    residuals = np.linalg.norm(AA_rot - BB, axis=1)

    scale = np.mean(np.linalg.norm(BB, axis=1)) + 1e-9
    yaw_err_rad = np.mean(residuals) / scale

    return yaw_rad, yaw_err_rad

def yaw_from_quat(q):
    """Extract planar yaw from geometry_msgs quaternion."""
    return np.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def yaw_to_quat(yaw):
    """Convert planar yaw to quaternion."""
    return (0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0))


def compute_planar_sigma(err_cm, base_var=0.01):
    """
    Compute planar position variance (sigma) in m^2 from reprojection error.
    err_cm: RMS error in cm
    base_var: minimum variance in m^2
    """
    return max(base_var, (err_cm * 0.01) ** 2)

def confidence_from_yaw_error(err_rad, sigma0=np.deg2rad(2.5), floor=1e-3):
    c = np.exp(-0.5 * (abs(err_rad) / sigma0) ** 2)
    return float(max(c, floor))


def covariance_from_sigma(sigma):
    """
    Construct a planar 6x6 covariance matrix from sigma (m^2).
    """
    cov = [0.0] * 36
    cov[0] = sigma        # x
    cov[7] = sigma        # y
    cov[35] = sigma * 0.5 # yaw (slightly more confident than position)
    return cov


def confidence_from_sigma(sigma, sigma_ref=0.01):
    """
    Map variance (sigma) to a confidence scalar in [0, 1].
    sigma_ref sets the variance where confidence ≈ e^-1.
    """
    return float(np.exp(-sigma / sigma_ref))