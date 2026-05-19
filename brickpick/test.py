"""Step-2 seed detection diagnostics.

Creates a dummy map, runs the seed-finding step of MapPartitioner, and
produces a coverage report identifying blind spots where the free space
is too far from any seed.
"""
import os
import yaml
import cv2
import numpy as np
from scipy import ndimage
from map_partitioner import MapPartitioner


def create_dummy_map_for_test(yaml_path):
    """Generate a synthetic occupancy grid with rooms, corridors, and corners."""
    pgm_path = yaml_path.replace('.yaml', '.pgm')
    img = np.full((500, 500), 254, dtype=np.uint8)

    # Outer walls
    img[0:10, :] = 0
    img[490:500, :] = 0
    img[:, 0:10] = 0
    img[:, 490:500] = 0

    # Narrow corridor (upper area)
    img[150:250, 240:260] = 0
    img[150:170, 240:450] = 0
    img[150:170, 430:450] = 0

    # Small rooms (lower right)
    img[250:260, 250:490] = 0
    img[350:360, 250:490] = 0
    img[250:360, 370:380] = 0

    # Unknown area
    img[400:450, 50:100] = 205

    cv2.imwrite(pgm_path, img)
    yaml_data = {
        'image': os.path.basename(pgm_path),
        'resolution': 0.05,
        'origin': [-12.5, -12.5, 0.0],
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f)


def evaluate_step2(yaml_path, home_pixel, inflation_radius, min_peak_distance):
    """Run step 2 and report seed coverage quality metrics."""
    print(f"\n{'=' * 50}")
    print(f"Step 2 evaluation (min_peak_distance={min_peak_distance})")
    print(f"{'=' * 50}")

    partitioner = MapPartitioner(
        yaml_path=yaml_path, inflation_radius=inflation_radius)
    partitioner.step1_preprocess(home_pixel=home_pixel)
    partitioner.step2_find_seeds()

    if len(partitioner.seeds) == 0:
        print("\n❌ No seeds found — algorithm cannot proceed.")
        return

    seeds = partitioner.seeds
    total_seeds = len(seeds)
    free_pixels = np.sum(partitioner.free_mask == 1)

    # Classify seeds as interior peaks vs. boundary-adjacent.
    kernel_small = np.ones((5, 5), np.uint8)
    eroded_mask = cv2.erode(partitioner.free_mask, kernel_small, iterations=1)
    boundary_band = partitioner.free_mask - eroded_mask

    is_boundary_seed = np.array(
        [boundary_band[y, x] > 0 for y, x in seeds])
    num_peaks = np.sum(~is_boundary_seed)
    num_boundary = np.sum(is_boundary_seed)

    # Compute distance to nearest seed for every free pixel.
    seed_mask = np.zeros_like(partitioner.free_mask, dtype=np.uint8)
    for y, x in seeds:
        seed_mask[y, x] = 1

    dist_to_nearest_seed = ndimage.distance_transform_edt(
        partitioner.free_mask - seed_mask)

    blind_threshold_pixels = 30
    blind_area_mask = (
        (dist_to_nearest_seed > blind_threshold_pixels)
        & (partitioner.free_mask == 1))
    blind_pixels = np.sum(blind_area_mask)
    blind_percentage = (
        (blind_pixels / free_pixels * 100) if free_pixels > 0 else 0)

    valid_dists = dist_to_nearest_seed[partitioner.free_mask == 1]
    avg_dist = np.mean(valid_dists)
    std_dist = np.std(valid_dists)

    print("\n【Step 2 Report】")
    print(f"  Total seeds: {total_seeds}")
    print(f"    - Interior peaks: {num_peaks}")
    print(f"    - Boundary samples: {num_boundary}")
    print(f"  Mean nearest-seed distance: {avg_dist:.1f} px")
    print(f"  Distance stddev: {std_dist:.1f} px (lower = more uniform)")
    print(f"  Blind area: {blind_pixels} px "
          f"({blind_percentage:.2f}% of free space)")

    print("\n【Diagnosis】")
    is_good = True

    if blind_percentage > 10.0:
        print(f"  ❌ Severe: {blind_percentage:.1f}% of free space > 1.5 m "
              f"from any seed.")
        print("     Suggestion: reduce min_peak_distance or add boundary "
              "samples.")
        is_good = False
    elif blind_percentage > 3.0:
        print(f"  ⚠️ Minor blind spots ({blind_percentage:.1f}%). "
              f"Consider reducing min_peak_distance.")
        is_good = False

    if std_dist > avg_dist * 0.8:
        print(f"  ⚠️ High non-uniformity (stddev {std_dist:.1f} ≈ mean "
              f"{avg_dist:.1f}).")
        print("     Suggestion: adjust boundary sampling step.")
        is_good = False

    if total_seeds > free_pixels / 500:
        print(f"  ⚠️ Over-dense seeds ({total_seeds}) — merging will be slow.")
        print("     Suggestion: increase min_peak_distance.")
        is_good = False

    if is_good:
        print("  ✅ Seed distribution is healthy for watershed.")

    # Generate diagnostic visualization.
    h, w = partitioner.free_mask.shape
    dist_norm = cv2.normalize(
        partitioner.dist_map, None, 0, 255, cv2.NORM_MINMAX)
    heatmap = cv2.applyColorMap(dist_norm.astype(np.uint8), cv2.COLORMAP_JET)

    gray_bg = np.full_like(heatmap, 128)
    vis = np.where(
        partitioner.free_mask[:, :, np.newaxis] == 1, heatmap, gray_bg)

    vis[blind_area_mask] = (0, 0, 255)

    for i, (y, x) in enumerate(seeds):
        color = (255, 255, 0) if is_boundary_seed[i] else (255, 255, 255)
        cv2.circle(vis, (x, y), 3, color, -1)

    cv2.circle(vis, (home_pixel[1], home_pixel[0]), 6, (0, 255, 0), -1)

    output_path = "test_step2_result.png"
    cv2.imwrite(output_path, vis)
    print(f"\n-> Visualization saved to: {output_path}")
    print("   Legend: heatmap(blue=narrow, red=wide) | red=blind spots | "
          "white=peak seed | yellow=boundary seed | green=home")


if __name__ == "__main__":
    yaml_file = 'sim/sample_map.yaml'

    if not os.path.exists(yaml_file):
        os.makedirs('sim', exist_ok=True)
        create_dummy_map_for_test(yaml_file)

    evaluate_step2(
        yaml_file, home_pixel=(80, 80),
        inflation_radius=5, min_peak_distance=20)
