"""Map partitioner using Voronoi watershed segmentation with A*-based merging.

Divides an occupancy grid into reachable regions seeded by distance-transform
peaks, merges small or high-cost regions greedily, and solves a nearest-neighbor
TSP over the final region centers to produce an ordered visitation sequence.
"""
import os
import yaml
import cv2
import numpy as np
import heapq
from scipy import ndimage
from skimage.segmentation import watershed
from skimage.segmentation import relabel_sequential
from skimage import graph
import math

import matplotlib.pyplot as plt


class MapPartitioner:
    def __init__(self, yaml_path, inflation_radius=3, min_area=500,
                 alpha=1.0, beta=0.002, merge_cost_threshold=1.5,
                 max_region_radius=1):
        self.yaml_path = yaml_path
        self.inflation_radius = inflation_radius
        self.min_area = min_area
        self.alpha = alpha
        self.beta = beta
        self.merge_cost_threshold = merge_cost_threshold
        self.max_region_radius = max_region_radius

        self._load_map()

    def _load_map(self):
        """Parse a ROS-format YAML + PGM map pair into an occupancy grid."""
        with open(self.yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        pgm_path = os.path.join(os.path.dirname(self.yaml_path),
                                config['image'])
        self.resolution = config['resolution']
        self.origin = config['origin']

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError(f"Cannot load map image: {pgm_path}")

        # ROS convention: 0 = occupied, 254/255 = free, 205 = unknown
        self.occ_map = np.zeros_like(img, dtype=np.uint8)
        self.occ_map[img < 250] = 1
        self.occ_map[img >= 250] = 0

        print(f"Map loaded. Size: {self.occ_map.shape}, "
              f"Resolution: {self.resolution}")

    def _astar(self, start, goal, binary_mask):
        """A* search on an 8-connected grid.

        Returns:
            list of (row, col) tuples from start to goal, or None if
            unreachable.
        """
        if not binary_mask[start[0], start[1]] or not binary_mask[goal[0], goal[1]]:
            return None

        def heuristic(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        closed_set = set()

        neighbors = [(-1, -1), (-1, 0), (-1, 1),
                     (0, -1),           (0, 1),
                     (1, -1),  (1, 0),  (1, 1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path

            if current in closed_set:
                continue
            closed_set.add(current)

            for dy, dx in neighbors:
                ny, nx = current[0] + dy, current[1] + dx
                neighbor = (ny, nx)

                in_bounds = (0 <= ny < binary_mask.shape[0]
                             and 0 <= nx < binary_mask.shape[1])
                if not in_bounds:
                    continue
                if binary_mask[ny, nx] == 0 or neighbor in closed_set:
                    continue

                move_cost = 1.414 if (dy != 0 and dx != 0) else 1.0
                tentative_g_score = g_score[current] + move_cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
        return None

    def visualize_occ_map(self, home_pixel=None,
                          save_path="occ_map_visualization.png"):
        """Render the occupancy grid to a side-by-side diagnostic image."""
        plt.figure(figsize=(10, 5))

        vis_map = np.ones(
            (self.occ_map.shape[0], self.occ_map.shape[1], 3), dtype=np.uint8) * 255
        vis_map[self.occ_map == 1] = [0, 0, 0]

        if home_pixel is not None:
            h, w = self.occ_map.shape
            if not (0 <= home_pixel[0] < h and 0 <= home_pixel[1] < w):
                print(f"⚠️ Warning: home_pixel {home_pixel} is out of "
                      f"map bounds {self.occ_map.shape}!")
                home_pixel = None

        plt.subplot(121)
        plt.title(f"Occupancy Map (Binary)\n"
                  f"0=Free(White), 1=Occupied(Black)\n"
                  f"Shape: {self.occ_map.shape}")
        plt.imshow(vis_map)
        if home_pixel is not None:
            plt.scatter([home_pixel[1]], [home_pixel[0]],
                        c='red', s=120, marker='x', linewidths=2.5,
                        label='Home')
            plt.legend(loc='upper right')
        plt.axis('off')

        plt.subplot(122)
        plt.title(f"Raw occ_map Values\nResolution: {self.resolution}m")
        plt.imshow(self.occ_map, cmap='gray', vmin=0, vmax=1)
        if home_pixel is not None:
            plt.scatter([home_pixel[1]], [home_pixel[0]],
                        c='red', s=120, marker='x', linewidths=2.5)
        plt.colorbar(label='Occupancy (0=Free, 1=Occupied)')
        plt.axis('off')

        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        print(f"✅ Occupancy map saved to {save_path}")
        plt.show()

        total_pixels = self.occ_map.size
        free_pixels = np.sum(self.occ_map == 0)
        occ_pixels = np.sum(self.occ_map == 1)
        print(f"\n=== Occupancy Map Statistics ===")
        print(f"Total pixels: {total_pixels}")
        print(f"Free space (0): {free_pixels} "
              f"({free_pixels / total_pixels * 100:.2f}%)")
        print(f"Occupied (1): {occ_pixels} "
              f"({occ_pixels / total_pixels * 100:.2f}%)")
        print(f"Map bounds: rows [0-{self.occ_map.shape[0]}], "
              f"cols [0-{self.occ_map.shape[1]}]")

        if home_pixel is not None:
            hp_val = self.occ_map[home_pixel[0], home_pixel[1]]
            print(f"Home pixel {home_pixel} value: {hp_val} "
                  f"(0=Free, 1=Occupied)")

    def step1_preprocess(self, home_pixel):
        """Inflate obstacles, then extract the connected free-space component
        reachable from the home position."""
        print("Step 1: Preprocessing map...")
        self.home_pixel = tuple(home_pixel)

        occ_mask = (self.occ_map == 1).astype(np.uint8)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.inflation_radius * 2 + 1, self.inflation_radius * 2 + 1))
        inflated_occ_mask = cv2.dilate(occ_mask, kernel, iterations=1)

        free_mask = (inflated_occ_mask == 0).astype(np.uint8)

        labeled_array, num_features = ndimage.label(free_mask)

        if labeled_array[self.home_pixel] == 0:
            raise ValueError("Home point is not in the free/inflated space!")

        self.free_mask = (
            labeled_array == labeled_array[self.home_pixel]).astype(np.uint8)
        print(f"  -> Extracted reachable free space. "
              f"Pixels: {np.sum(self.free_mask)}")

    def step2_find_seeds(self, phys_peak_distance=0.3):
        """Detect local maxima of the distance transform as region seeds."""
        print("Step 2: Finding seed points...")

        if not hasattr(self, 'resolution') or self.resolution <= 0:
            raise ValueError(
                "Map resolution not found. Did step1 run correctly?")

        min_dist_pixels = int(phys_peak_distance / self.resolution)

        dist_map = ndimage.distance_transform_edt(self.free_mask)
        self.dist_map = dist_map

        from skimage.feature import peak_local_max
        peaks = peak_local_max(
            dist_map,
            min_distance=min_dist_pixels,
            labels=self.free_mask,
            exclude_border=False,
        )

        self.seeds = peaks
        print(f"  -> Physical min distance: {phys_peak_distance}m "
              f"({min_dist_pixels} px)")
        print(f"  -> Found {len(self.seeds)} core seed points.")

    def step3_voronoi_partition(self):
        """Partition free space via marker-based watershed on the inverted
        distance transform."""
        print("Step 3: Generating Voronoi partitions...")
        if len(self.seeds) == 0:
            raise ValueError("No seed points found!")

        markers = np.zeros(self.free_mask.shape, dtype=np.int32)
        for i, (y, x) in enumerate(self.seeds):
            markers[y, x] = i + 1

        self.labels = watershed(-self.dist_map, markers, mask=self.free_mask)
        self.num_regions_init = np.max(self.labels)
        print(f"  -> Generated {self.num_regions_init} initial regions.")

    def step4_optimize_and_merge(self):
        """Merge small regions greedily with a max-radius hard constraint."""
        print("Step 4: Optimizing and merging regions...")
        labels = self.labels

        # Merge regions below the minimum area threshold.
        unique_labels, counts = np.unique(labels, return_counts=True)
        label_area_map = dict(zip(unique_labels, counts))
        small_labels = [l for l, area in label_area_map.items()
                        if area < self.min_area and l != 0]
        for sl in small_labels:
            dilated = ndimage.binary_dilation(labels == sl)
            neighbors = np.unique(labels[dilated])
            neighbors = [n for n in neighbors if n != 0 and n != sl]
            if neighbors:
                max_neighbor = max(neighbors,
                                   key=lambda n: label_area_map.get(n, 0))
                labels[labels == sl] = max_neighbor
                label_area_map[max_neighbor] += label_area_map.get(sl, 0)
                if sl in label_area_map:
                    del label_area_map[sl]

        labels, _, _ = relabel_sequential(labels)
        regions_props = ndimage.find_objects(labels)
        region_centers = {}
        region_path_dists = {}
        region_max_radii_px = {}

        print("  -> Calculating path distances and MAX RADII to region "
              "centers...")
        num_regions = np.max(labels)

        max_radius_px_limit = self.max_region_radius / self.resolution

        for sl in range(1, num_regions + 1):
            slice_obj = regions_props[sl - 1]
            if slice_obj is None:
                continue
            y_coords, x_coords = np.where(labels[slice_obj] == sl)
            if len(y_coords) == 0:
                continue

            cy = int(np.mean(y_coords) + slice_obj[0].start)
            cx = int(np.mean(x_coords) + slice_obj[1].start)
            cy, cx = self._clamp_to_free(cy, cx)
            region_centers[sl] = (cy, cx)

            path = self._astar(self.home_pixel, (cy, cx), self.free_mask)
            region_path_dists[sl] = len(path) if path else float('inf')

            dists_to_center = np.sqrt(
                (y_coords - cy) ** 2 + (x_coords - cx) ** 2)
            region_max_radii_px[sl] = (
                np.max(dists_to_center) if len(dists_to_center) > 0 else 0)

        # Greedy merge loop with radius constraint.
        merged = True
        while merged:
            merged = False
            min_cost = float('inf')
            best_edge = None

            rag = graph.RAG(labels, connectivity=2)

            def merge_cost(r1, r2):
                area1 = np.sum(labels == r1)
                area2 = np.sum(labels == r2)
                if area1 == 0 or area2 == 0:
                    return float('inf')

                # Hard radius constraint: reject merges that would
                # exceed max_region_radius.
                c1 = region_centers.get(r1)
                c2 = region_centers.get(r2)
                if c1 and c2:
                    center_dist = math.sqrt(
                        (c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)
                    estimated_merged_radius = (
                        center_dist
                        + max(region_max_radii_px.get(r1, 0),
                              region_max_radii_px.get(r2, 0)))

                    if estimated_merged_radius > max_radius_px_limit:
                        return float('inf')

                area_cost = (1.0 / area1) + (1.0 / area2)
                dist_cost = (region_path_dists.get(r1, float('inf'))
                             + region_path_dists.get(r2, float('inf')))
                return self.alpha * area_cost + self.beta * dist_cost

            for (r1, r2) in rag.edges():
                if r1 == 0 or r2 == 0:
                    continue
                cost = merge_cost(r1, r2)
                if cost < min_cost:
                    min_cost = cost
                    best_edge = (r1, r2)

            if best_edge and min_cost < self.merge_cost_threshold:
                r1, r2 = best_edge
                labels[labels == r2] = r1

                d1 = region_path_dists.get(r1, float('inf'))
                d2 = region_path_dists.get(r2, float('inf'))
                region_path_dists[r1] = min(d1, d2)
                if r2 in region_path_dists:
                    del region_path_dists[r2]

                # Recompute the true max radius for the merged region.
                y_new, x_new = np.where(labels == r1)
                if len(y_new) > 0:
                    c_new = region_centers[r1]
                    new_dists = np.sqrt(
                        (y_new - c_new[0]) ** 2 + (x_new - c_new[1]) ** 2)
                    region_max_radii_px[r1] = np.max(new_dists)
                if r2 in region_max_radii_px:
                    del region_max_radii_px[r2]

                merged = True

        self.final_labels, _, _ = relabel_sequential(labels)
        self.final_num_regions = np.max(self.final_labels)
        print(f"  -> Merged into {self.final_num_regions} final regions "
              f"(Respecting {self.max_region_radius}m radius limit).")

    def step5_calculate_centers(self):
        """Compute reachable region centers and order them via greedy TSP."""
        print("Step 5: Calculating reachable centers and solving TSP...")
        self.reachable_centers = []
        regions_props = ndimage.find_objects(self.final_labels)

        raw_centers = []
        for sl in range(1, self.final_num_regions + 1):
            slice_obj = regions_props[sl - 1]
            if slice_obj is None:
                continue
            y_coords, x_coords = np.where(self.final_labels[slice_obj] == sl)

            cy = int(np.mean(y_coords) + slice_obj[0].start)
            cx = int(np.mean(x_coords) + slice_obj[1].start)

            path = self._astar(self.home_pixel, (cy, cx), self.free_mask)

            if path is None:
                # Back-off along the ray from home toward the unreachable
                # centroid until a reachable pixel in the same region is found.
                direction_y = cy - self.home_pixel[0]
                direction_x = cx - self.home_pixel[1]
                length = math.sqrt(direction_y ** 2 + direction_x ** 2)
                if length > 0:
                    for step in np.linspace(0, length, num=int(length)):
                        test_y = int(self.home_pixel[0]
                                     + (direction_y / length) * step)
                        test_x = int(self.home_pixel[1]
                                     + (direction_x / length) * step)
                        test_y, test_x = self._clamp_to_free(test_y, test_x)

                        if self.final_labels[test_y, test_x] == sl:
                            test_path = self._astar(
                                self.home_pixel, (test_y, test_x),
                                self.free_mask)
                            if test_path is not None:
                                path = test_path
                                cy, cx = test_y, test_x
                                break

            if path is not None:
                raw_centers.append((cy, cx))
            else:
                print(f"    -> Warning: Region {sl} center unreachable, "
                      f"skipped.")

        n = len(raw_centers)
        if n == 0:
            return

        print(f"  -> Found {n} valid centers. "
              f"Calculating pairwise distances for TSP...")

        # Pairwise A* distance matrix.
        dist_matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                path = self._astar(
                    raw_centers[i], raw_centers[j], self.free_mask)
                d = len(path) if path else float('inf')
                dist_matrix[i][j] = d
                dist_matrix[j][i] = d

        home_dists = []
        for i in range(n):
            path = self._astar(
                self.home_pixel, raw_centers[i], self.free_mask)
            home_dists.append(len(path) if path else float('inf'))

        # Nearest-neighbor greedy TSP from home.
        visited = [False] * n
        ordered_centers = []
        current_idx = -1
        total_path_cost = 0

        for _ in range(n):
            best_next_idx = -1
            best_next_dist = float('inf')

            for i in range(n):
                if visited[i]:
                    continue

                if current_idx == -1:
                    d = home_dists[i]
                else:
                    d = dist_matrix[current_idx][i]

                if d < best_next_dist:
                    best_next_dist = d
                    best_next_idx = i

            if best_next_idx == -1:
                break

            visited[best_next_idx] = True
            ordered_centers.append(raw_centers[best_next_idx])
            total_path_cost += best_next_dist
            current_idx = best_next_idx

        self.reachable_centers = ordered_centers
        print(f"  -> TSP routing complete. "
              f"Total estimated path cost: {total_path_cost:.0f} pixels.")

    def _clamp_to_free(self, y, x):
        """Snap an out-of-bounds or occupied pixel to the nearest free cell."""
        h, w = self.free_mask.shape
        y, x = np.clip(y, 0, h - 1), np.clip(x, 0, w - 1)
        if self.free_mask[y, x] == 0:
            for r in range(1, 10):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < h and 0 <= nx < w
                                and self.free_mask[ny, nx] == 1):
                            return ny, nx
        return y, x

    def pixel_to_world(self, py, px):
        """Convert pixel coordinates to world frame (ROS Y-axis points up)."""
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.free_mask.shape[0] - py) * self.resolution
        return (wx, wy)

    def world_to_pixel(self, wx, wy):
        """Convert world coordinates to pixel indices."""
        dx = wx - self.origin[0]
        dy = wy - self.origin[1]
        px = int(dx / self.resolution)
        py = int(self.occ_map.shape[0] - (dy / self.resolution))
        return (py, px)

    def process(self, home_pixel):
        """Run the full partitioning pipeline and return ordered centers."""
        self.visualize_occ_map(home_pixel)
        self.step1_preprocess(home_pixel)
        self.step2_find_seeds()
        self.step3_voronoi_partition()
        self.step4_optimize_and_merge()
        self.step5_calculate_centers()
        return self.reachable_centers

    def save_results(self, output_img_path, output_txt_path):
        """Write a color-coded partition image and a world-coordinate text file."""
        print(f"Saving results to {output_img_path} and {output_txt_path}...")

        vis_img = np.ones(
            (self.free_mask.shape[0], self.free_mask.shape[1], 3),
            dtype=np.uint8) * 255
        vis_img[self.occ_map == 1] = (0, 0, 0)

        np.random.seed(42)
        colors = np.random.randint(
            50, 255, size=(self.final_num_regions + 1, 3))
        colors[0] = [255, 255, 255]

        for sl in range(1, self.final_num_regions + 1):
            vis_img[self.final_labels == sl] = colors[sl]

        for y, x in self.seeds:
            cv2.circle(vis_img, (x, y), 2, (255, 0, 0), -1)

        cv2.circle(vis_img, (self.home_pixel[1], self.home_pixel[0]),
                   6, (0, 255, 0), -1)

        for i, (cy, cx) in enumerate(self.reachable_centers):
            cv2.drawMarker(vis_img, (cx, cy), (0, 0, 255),
                           markerType=cv2.MARKER_CROSS,
                           markerSize=15, thickness=2)
            cv2.putText(vis_img, str(i), (cx + 5, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imwrite(output_img_path, vis_img)

        with open(output_txt_path, 'w') as f:
            f.write("# Region Centers (World Coordinates: x y)\n")
            for cy, cx in self.reachable_centers:
                wx, wy = self.pixel_to_world(cy, cx)
                f.write(f"{wx:.3f} {wy:.3f}\n")

        print("Done!")


def visualize_step2_details(self, phys_peak_distance=0.6):
    """Diagnostic helper: plot distance transform and detected peaks."""
    print("Visualizing Step 2 details...")

    min_dist_pixels = int(phys_peak_distance / self.resolution)
    dist_map = ndimage.distance_transform_edt(self.free_mask)

    from skimage.feature import peak_local_max
    peaks = peak_local_max(
        dist_map,
        min_distance=min_dist_pixels,
        labels=self.free_mask,
        exclude_border=False,
    )

    plt.figure(figsize=(18, 5))

    plt.subplot(131)
    plt.title("1. Reachable Free Space\n(Input for Step 2)")
    plt.imshow(self.free_mask, cmap='gray')
    plt.axis('off')

    plt.subplot(132)
    plt.title("2. Distance Transform\n(The 'Mountain' Map)")
    im2 = plt.imshow(dist_map, cmap='viridis')
    plt.colorbar(im2, fraction=0.046, pad=0.04)
    plt.axis('off')

    plt.subplot(133)
    plt.title(f"3. Local Maxima Extracted\n(Seeds, dist={phys_peak_distance}m)")
    plt.imshow(dist_map, cmap='magma')
    plt.plot(peaks[:, 1], peaks[:, 0], 'r.', markersize=8, label='Seeds')
    plt.legend()
    plt.axis('off')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    yaml_file = 'map/livingroom_clean_12.yaml'

    partitioner = MapPartitioner(
        yaml_path=yaml_file,
        inflation_radius=3,
        min_area=300,
        alpha=1.0,
        beta=0.002,
        merge_cost_threshold=2.0,
    )

    home_py, home_px = 288, 48

    print(f"Origin from YAML: {partitioner.origin[:2]}")
    print(f"Calculated Home Pixel: {home_py, home_px}")

    centers = partitioner.process(home_pixel=(home_py, home_px))

    partitioner.save_results(
        output_img_path="partitioned_map.png",
        output_txt_path="region_centers.txt",
    )
