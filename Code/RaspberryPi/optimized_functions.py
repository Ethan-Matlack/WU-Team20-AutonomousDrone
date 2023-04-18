import numpy as np
from scipy.spatial.distance import cdist

def contour_compare_spatial(contour_list_a, contour_list_b, search_radius):
    contour_list_common = []
    for contour_a in contour_list_a:
        contour_a = cv.approxPolyDP(contour_a, 30, True)
        for contour_b in contour_list_b:
            contour_b = cv.approxPolyDP(contour_b, 30, True)
            # This might rely on arrays being the same size...
            dist = cdist(contour_a.reshape(-1, 2), contour_b.reshape(-1, 2))
            mask = (dist <= search_radius)
            if np.any(mask):
                # Get the indices of matched points
                idxs_a, idxs_b = np.where(mask)
                # Calculate the average of matched points
                avg = np.round((contour_a[idxs_a] + contour_b[idxs_b]) / 2).astype(int)
                contour_list_common.extend(avg.tolist())
    return contour_list_common

# =====================================================================================================================
# =====================================================================================================================

# Create four lists... CM MY YK KC
# Each list contains a tuple (x, y) and a tag "color"

def find_nearby_pairs(coords, categories, radius):
    # Split coords into subarrays by category
    subarrays = [coords[categories == i] for i in range(4)]

    # Build a KDTree for each subarray
    kdtrees = [KDTree(subarray[:, :2]) for subarray in subarrays]

    pairs = []
    for i, subarray in enumerate(subarrays):
        # Query the other KDTrees to find nearby points
        for j in range(i + 1, 4):
            other_subarray = subarrays[j]
            other_kdtree = kdtrees[j]

            # Query the KDTree for nearby points
            nearby_idxs = other_kdtree.query_ball_point(subarray[:, :2], radius)

            # Check if the nearby points have a different category
            for idx, nearby_idx_list in enumerate(nearby_idxs):
                for nearby_idx in nearby_idx_list:
                    if categories[subarray[idx, 2]] != categories[other_subarray[nearby_idx, 2]]:
                        pairs.append((subarray[idx, :2], other_subarray[nearby_idx, :2]))

    return pairs

from scipy.spatial import KDTree

def find_nearby_pairs_kdtree(coords, categories, radius):
    # Split coords into subarrays by category
    subarrays = [coords[categories == i] for i in range(4)]

    # Build a KDTree for each subarray
    kdtrees = [KDTree(subarray[:, :2]) for subarray in subarrays]

    pairs = []
    for i, subarray in enumerate(subarrays):
        # Query the other KDTrees to find nearby points
        for j in range(i + 1, 4):
            other_subarray = subarrays[j]
            other_kdtree = kdtrees[j]

            # Query the KDTree for nearby points
            nearby_idxs = other_kdtree.query_ball_point(subarray[:, :2], radius)

            # Check if the nearby points have a different category
            for idx, nearby_idx_list in enumerate(nearby_idxs):
                for nearby_idx in nearby_idx_list:
                    if categories[subarray[idx, 2]] != categories[other_subarray[nearby_idx, 2]]:
                        pairs.append((subarray[idx, :2], other_subarray[nearby_idx, :2]))

    return pairs
