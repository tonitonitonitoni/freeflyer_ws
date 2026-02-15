from combined_pose_system.utils.pose_system_core import *
from dataclasses import dataclass
import os

def safe_normalize(v, eps=1e-9):
    """Vectorized version of safe_normalize_num"""
    v = np.asarray(v, dtype=float)

    if v.ndim == 1:
        n = np.linalg.norm(v)
        if n < eps:
            print("zeros returned")
            return np.zeros_like(v)
        return v / n

    norms = np.linalg.norm(v, axis=-1, keepdims=True)
    mask = norms < eps
    if np.any(mask):
        print(f"zeros returned for {len(np.nonzero(mask))} values")
    safe_norms = np.where(mask, 1.0, norms)
    out = v / safe_norms
    return np.where(mask, 0.0, out)

def pixels_to_rays_camera(uv_px, K, dist=None):
    """
    uv_px: (N,2) pixel coords (u,v)
    K: (3,3)
    dist: (k,) distortion coefficients or None

    returns:
      rays_c: (N,3) unit rays in camera frame
      xy_n: (N,2) normalized coords (x_n, y_n)
    """
    uv = np.asarray(uv_px, dtype=np.float32).reshape(-1, 1, 2)

    if dist is not None:
        # returns normalized points by default (x_n, y_n)
        xy = cv2.undistortPoints(uv, K, dist)  # (N,1,2)
        xy = xy.reshape(-1, 2)
    else:
        fx, fy = K[0,0], K[1,1]
        cx, cy = K[0,2], K[1,2]
        u = uv.reshape(-1, 2)[:,0]
        v = uv.reshape(-1, 2)[:,1]
        xy = np.stack([(u - cx)/fx, (v - cy)/fy], axis=1)

    rays = np.concatenate([xy, np.ones((xy.shape[0], 1), dtype=np.float32)], axis=1)
    rays = safe_normalize(rays)#rays / (np.linalg.norm(rays, axis=1, keepdims=True) + 1e-12)
    return rays.astype(np.float64), xy.astype(np.float64)

def R_wc_from_yaw(yaw_rad):
    """
    Camera frame convention matters.
    This assumes:
      - world Z up
      - yaw rotates around world Z
      - camera frame x right, y down, z forward (OpenCV-ish)
    You may need a fixed alignment rotation between your camera axes and world axes.
    """
    c = np.cos(yaw_rad)
    s = np.sin(yaw_rad)
    Rz = np.array([[ c, -s, 0],
                   [ s,  c, 0],
                   [ 0,  0, 1]], dtype=np.float64)
    return Rz

def intersect_rays_with_plane(origins_w, dirs_w, n_w, d):
    """
    Plane: n^T X + d = 0
    origins_w, dirs_w: (N,3)

    returns:
      X: (N,3) intersection points
      valid: (N,) boolean
    """
    n_w = safe_normalize(np.asarray(n_w, dtype=np.float64).reshape(3,))
    o = origins_w
    v = dirs_w

    denom = v @ n_w  # (N,)
    numer = -(o @ n_w + d)  # (N,)

    # valid if denom not tiny and lambda positive
    eps = 1e-9
    valid = np.abs(denom) > eps
    lam = np.zeros_like(numer)
    lam[valid] = numer[valid] / denom[valid]
    valid = valid & (lam > 0)

    X = o + lam[:, None] * v
    return X, valid

#Observation structures
@dataclass
class FrameObs:
    frame_idx: int
    cam_pos_w: np.ndarray      # (3,)
    yaw_rad: float
    centroids_px: np.ndarray   # (N,3): x,y,color
    cluster_ids: list[int]     # len N

@dataclass
class RayObs:
    frame_idx: int
    cam_pos_w: np.ndarray
    ray_dir_w: np.ndarray      # (3,)

# Step 1 — video → per-frame observations
def extract_frame_observations(video_path, pose_log):
    """
    pose_log[frame_idx] = (cam_pos_w, yaw_rad)
    """
    cap = cv2.VideoCapture(video_path)
    frame_idx = 0
    observations = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        centroids = find_centroids(frame)
        if len(centroids) == 0:
            frame_idx += 1
            continue

        patterns, _ = find_patterns(centroids)

        beacon_pos_w, yaw = pose_log[frame_idx]
        # Beacon to camera translation in metres
        t_bc = np.array([
            0.12,   # x forward
            0.00,   # y left
            0.05,   # z up
        ])
        cam_pos_w = beacon_pos_w + R_wc_from_yaw(yaw) @ t_bc

        for pat in patterns:
            cid = identify_cluster(pat)
            if cid < 0:
                continue

            # sort so LED ordering is consistent inside the cluster
            sorted_pts = sort_cluster(pat)

            for local_led_idx, (x, y, color) in enumerate(sorted_pts): #type:ignore
                global_led_id = 3 * cid + local_led_idx

                observations.append(FrameObs(
                    frame_idx=frame_idx,
                    cam_pos_w=cam_pos_w,
                    yaw_rad=yaw,
                    centroids_px=np.array([[x, y, color]]),
                    cluster_ids=[global_led_id]
                ))

        frame_idx += 1

    return observations

# Step 2 — centroids → world rays
def centroids_to_camera_rays(centroids_px, K, dist):
    pts = centroids_px[:, :2].astype(np.float32)
    pts = pts.reshape(-1, 1, 2)

    xy = cv2.undistortPoints(pts, K, dist)  # (N,1,2)
    xy = xy.reshape(-1, 2)

    rays = np.hstack([xy, np.ones((xy.shape[0], 1))])
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return rays

def rays_world_from_pose(rays_c, p_w, yaw_rad, R_align=None):
    """
    rays_c: (N,3) in camera frame
    p_w: (3,) camera position in world
    yaw_rad: scalar
    R_align: (3,3) fixed rotation mapping camera axes into your chosen "body/world" convention.
             If None, assumes camera axes already consistent.
    returns:
      origins_w: (N,3) all == p_w
      dirs_w: (N,3) unit directions in world
    """
    p_w = np.asarray(p_w, dtype=np.float64).reshape(3,)
    R = R_wc_from_yaw(yaw_rad)

    if R_align is not None:
        # first map rays into some body/world-aligned camera frame
        rays_c2 = (R_align @ rays_c.T).T
    else:
        rays_c2 = rays_c

    dirs_w = (R @ rays_c2.T).T
    dirs_w = dirs_w / (np.linalg.norm(dirs_w, axis=1, keepdims=True) + 1e-12)
    origins_w = np.repeat(p_w[None, :], dirs_w.shape[0], axis=0)
    return origins_w, dirs_w

def frame_obs_to_rays(frame_obs, K, dist, R_align=None):
    rays_c = centroids_to_camera_rays(frame_obs.centroids_px, K, dist)

    origins_w, dirs_w = rays_world_from_pose(
        rays_c,
        frame_obs.cam_pos_w,
        frame_obs.yaw_rad,
        R_align=R_align
    )

    return origins_w, dirs_w

# Step 3 — intersect rays with the board plane 
def intersect_with_board(origins_w, dirs_w, z0=4.5):
    n = np.array([0.0, 0.0, 1.0])
    d = -z0
    X, valid = intersect_rays_with_plane(origins_w, dirs_w, n, d)
    return X[valid]
# Step 4 — accumulate landmark observations
def accumulate_landmarks(frame_observations, K, dist, z0):
    landmark_pts = {i: [] for i in range(24)}

    for obs in frame_observations:
        origins_w, dirs_w = frame_obs_to_rays(obs, K, dist)

        pts_w = intersect_with_board(origins_w, dirs_w, z0)

        if len(pts_w) == 0:
            continue

        led_id = obs.cluster_ids[0]
        landmark_pts[led_id].append(pts_w[0])

    return landmark_pts

# Step 5 — estimate LED positions (first-pass map)
def estimate_landmark_positions(landmark_pts):
    landmark_pos = {}

    for lid, pts in landmark_pts.items():
        if len(pts) < 3:
            continue
        pts = np.array(pts)
        landmark_pos[lid] = np.mean(pts, axis=0)

    return landmark_pos

# Step 6 — optional yaw refinement 

# once you have tentative landmark positions, each observation gives you a yaw residual.

# For one ray:
# expected_dirn = normalize(L_i-p_w) where L_i = landmark location and p_w is cam position in world

def yaw_residual(ray_c, cam_pos_w, yaw, landmark_pos, R_align=None):
    # predicted direction in world
    r_pred = safe_normalize(landmark_pos - cam_pos_w)

    # actual direction from camera ray
    r_c2 = ray_c
    if R_align is not None:
        r_c2 = R_align @ r_c2

    # rotate ray by yaw
    r_obs = R_wc_from_yaw(yaw) @ r_c2

    # yaw error is angle between projections onto XY plane
    a = np.arctan2(r_pred[1], r_pred[0])
    b = np.arctan2(r_obs[1], r_obs[0])
    return a - b

def plot_starfield_map(landmark_pos):
    pts = np.array(list(landmark_pos.values()))
    plt.scatter(pts[:,0], pts[:,1])
    for i, p in landmark_pos.items():
        plt.text(p[0], p[1], str(i))
    plt.axis("equal")
    plt.title("Starfield Map (world frame)")
    plt.show()

#Let's test it and wire it up (with fake data for now)
def make_fake_pose_log(num_frames):
    """
    Fake camera motion for testing:
    camera moves in a gentle arc in front of the board.
    """
    pose_log = {}

    for k in range(num_frames):
        t = k / num_frames

        x = 1.0 + 0.5 * np.cos(2*np.pi*t)
        y = 0.5 * np.sin(2*np.pi*t)
        z = 1.2

        yaw = 0.2 * np.sin(2*np.pi*t)  # deliberately imperfect

        pose_log[k] = (np.array([x, y, z]), yaw)

    return pose_log

def load_pose_log(pose_dir):
    """
    Load pose log recorded by StarfieldSyncNode.

    Returns:
        pose_log : dict[int] -> (cam_pos_w, yaw_rad)
    """
    pose_log = {}

    files = sorted(f for f in os.listdir(pose_dir) if f.endswith(".npz"))
    for k, fname in enumerate(files):
        data = np.load(os.path.join(pose_dir, fname))
        pose_log[k] = (
            data["cam_pos_w"],
            float(data["yaw"]),
        )

    return pose_log


def build_starfield_map_from_video(
    video_path,
    K,
    dist,
    z0=4.5,
    R_align=None,
    max_frames=None,
    debug=False,
):
    """
    Offline starfield map builder.

    Args:
        video_path: path to recorded video
        K, dist: camera calibration
        z0: board plane height in world frame
        R_align: optional fixed camera->world axis alignment
        max_frames: limit frames for testing
        debug: verbose prints

    Returns:
        landmark_pos: dict {led_id: (3,)}
        landmark_pts: dict {led_id: list of (3,)}
    """

    # -----------------------------
    # Open video
    # -----------------------------
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if max_frames is not None:
        total_frames = min(total_frames, max_frames)

    print(f"[MAP] Processing {total_frames} frames")

    cap.release()  # extract_frame_observations re-opens it

    # -----------------------------
    # Pose stream (mock for now)
    # -----------------------------
    # pose_log = make_fake_pose_log(total_frames)
    pose_log = load_pose_log("logged_poses")


    # -----------------------------
    # Extract per-frame LED observations
    # -----------------------------
    frame_observations = extract_frame_observations(
        video_path=video_path,
        pose_log=pose_log,
    )

    print(f"[MAP] Extracted {len(frame_observations)} LED observations")

    # -----------------------------
    # Accumulate landmark samples
    # -----------------------------
    landmark_pts = accumulate_landmarks(
        frame_observations,
        K=K,
        dist=dist,
        z0=z0,
    )

    if debug:
        for lid, pts in landmark_pts.items():
            if len(pts) > 0:
                print(f"  LED {lid}: {len(pts)} samples")

    # -----------------------------
    # Estimate landmark positions
    # -----------------------------
    landmark_pos = estimate_landmark_positions(landmark_pts)

    print(f"[MAP] Estimated {len(landmark_pos)} landmarks")

    return landmark_pos, landmark_pts

def load_logged_dataset(frames_dir, poses_dir, max_frames=None):
    """
    Load logged JPEG frames and pose npz files indexed by frame number.

    Returns:
        frames     : dict[int] -> BGR image
        pose_log   : dict[int] -> (cam_pos_w, yaw_rad)
    """
    frames = {}
    pose_log = {}

    frame_files = sorted(
        f for f in os.listdir(frames_dir) if f.endswith(".jpg")
    )

    if max_frames is not None:
        frame_files = frame_files[:max_frames]

    for fname in frame_files:
        idx = int(fname.split("_")[1].split(".")[0])

        img_path = os.path.join(frames_dir, fname)
        pose_path = os.path.join(poses_dir, f"pose_{idx:06d}.npz")

        if not os.path.exists(pose_path):
            print(f"[WARN] Missing pose for frame {idx}")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print(f"[WARN] Failed to load frame {idx}")
            continue

        data = np.load(pose_path)
        cam_pos_w = data["cam_pos_w"]
        yaw = float(data["yaw"])

        frames[idx] = img
        pose_log[idx] = (cam_pos_w, yaw)

    print(f"[DATA] Loaded {len(frames)} synced frames")

    return frames, pose_log


if __name__ == "__main__":
    # --------------------------------
    # Fake calibration (replace later)
    # --------------------------------
    K = np.array([
        [800.0,   0.0, 640.0],
        [  0.0, 800.0, 360.0],
        [  0.0,   0.0,   1.0],
    ])
    dist = np.zeros(5)


    # video_path = "not_real.mp4"  # placeholder

    # try:
    #     landmark_pos, landmark_pts = build_starfield_map_from_video(
    #         video_path,
    #         K=K,
    #         dist=dist,
    #         z0=4.5,
    #         debug=True,
    #         max_frames=300,
    #     )

        # plot_starfield_map(landmark_pos)

    # except RuntimeError as e:
    #     print(e)
    #     print("Video not found — pipeline wiring is correct.")