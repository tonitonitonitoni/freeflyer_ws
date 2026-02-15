import cv2, json
from combined_pose_system.utils.utils_pose import find_centroids, find_patterns, identify_cluster
map_path = "/Users/antoniahoffman/PycharmProjects/starfieldProject/final_sub/master_map.png"

master_map = cv2.imread(map_path)
H = master_map.shape[0]
cens_all = find_centroids(master_map, visualize=True, area_threshold=50, dist_threshold=15)
patterns_all, num_clusters_all = find_patterns(cens_all)
#visualize_patterns(master_map, patterns_all, num_clusters_all, fid=0)
cluster_locs = {}
for pat in patterns_all:
    observed_points_img = pat[:, 0:2].tolist()

    # Convert from image coordinates (origin top-left, y down)
    # to planar coordinates (origin bottom-left, y up)
    observed_points = [
        [x, H - y] for (x, y) in observed_points_img
    ]
    cluster_index = identify_cluster(pat)
    cluster_locs[cluster_index] = observed_points

# Save to file
with open('cluster_data_jan_10.json', 'w') as f:
    json.dump(cluster_locs, f, indent=4)