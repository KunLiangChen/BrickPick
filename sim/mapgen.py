"""Living-room map generator and partitioner demo.

Generates a synthetic furnished apartment map, then runs the full map
partitioning pipeline to produce a set of reachable region centers.
"""
import numpy as np
import cv2
import os
import yaml
import heapq
from scipy import ndimage
from skimage.segmentation import watershed
from skimage import graph
from skimage.segmentation import relabel_sequential
import math


class MapPartitioner:
    """Voroni-watershed map partitioner with A*-based region merging.

    Variant tuned for synthetic maps: uses a 200-threshold occupancy
    model, boundary-seed augmentation, and simpler merge logic without
    the max-radius constraint.
    """

    def __init__(self, yaml_path, inflation_radius=3, min_area=500,
                 alpha=1.0, beta=0.002, merge_cost_threshold=1.5):
        self.yaml_path = yaml_path
        self.inflation_radius = inflation_radius
        self.min_area = min_area
        self.alpha = alpha
        self.beta = beta
        self.merge_cost_threshold = merge_cost_threshold
        self._load_map()

    def _load_map(self):
        with open(self.yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        pgm_path = os.path.join(os.path.dirname(self.yaml_path),
                                config['image'])
        self.resolution = config['resolution']
        self.origin = config['origin']

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError(f"Cannot load map image: {pgm_path}")

        self.occ_map = np.zeros_like(img, dtype=np.uint8)
        self.occ_map[img < 200] = 1
        self.occ_map[img >= 200] = 0
        self.unknown_mask = (img > 100) & (img < 200)
        print(f"Map loaded. Size: {self.occ_map.shape}, "
              f"Resolution: {self.resolution}")

    def _astar(self, start, goal, binary_mask):
        """A* on an 8-connected grid."""
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
                if 0 <= ny < binary_mask.shape[0] and 0 <= nx < binary_mask.shape[1]:
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

    def step1_preprocess(self, home_pixel):
        """Dilate free space and extract the home-connected component."""
        print("Step 1: Preprocessing map...")
        self.home_pixel = tuple(home_pixel)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.inflation_radius * 2 + 1, self.inflation_radius * 2 + 1))
        free_mask = (self.occ_map == 0).astype(np.uint8)
        inflated_mask = cv2.dilate(free_mask, kernel, iterations=1)
        labeled_array, num_features = ndimage.label(inflated_mask)
        if labeled_array[self.home_pixel] == 0:
            raise ValueError("Home point is not in the free/inflated space!")
        self.free_mask = (
            labeled_array == labeled_array[self.home_pixel]).astype(np.uint8)
        print(f"  -> Extracted reachable free space. "
              f"Pixels: {np.sum(self.free_mask)}")

    def step2_find_seeds(self, min_peak_distance=20):
        """Detect distance-transform peaks and augment with boundary samples."""
        print("Step 2: Finding seed points...")
        dist_map = ndimage.distance_transform_edt(self.free_mask)
        self.dist_map = dist_map
        from skimage.feature import peak_local_max
        peaks = peak_local_max(
            dist_map, min_distance=min_peak_distance, labels=self.free_mask)
        kernel_small = np.ones((5, 5), np.uint8)
        eroded_mask = cv2.erode(self.free_mask, kernel_small, iterations=1)
        boundary_band = self.free_mask - eroded_mask
        boundary_coords = np.argwhere(boundary_band > 0)
        if len(boundary_coords) > 0:
            step = max(1, len(boundary_coords) // 50)
            boundary_seeds = boundary_coords[::step]
        else:
            boundary_seeds = np.empty((0, 2), dtype=int)
        if len(peaks) > 0 and len(boundary_seeds) > 0:
            all_seeds = np.vstack((peaks, boundary_seeds))
        else:
            all_seeds = peaks if len(peaks) > 0 else boundary_seeds
        self.seeds = all_seeds
        print(f"  -> Found {len(self.seeds)} seed points.")

    def step3_voronoi_partition(self):
        """Watershed segmentation seeded by the selected points."""
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
        """Greedy region merging to eliminate small fragments."""
        print("Step 4: Optimizing and merging regions...")
        labels = self.labels
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
        labels, _, _ = relabel_sequential(labels)
        regions_props = ndimage.find_objects(labels)
        region_centers = {}
        region_path_dists = {}
        print("  -> Calculating path distances to region centers (A*)...")
        for i, sl in enumerate(range(1, np.max(labels) + 1)):
            slice_obj = regions_props[sl - 1]
            y_coords, x_coords = np.where(labels[slice_obj] == sl)
            cy = int(np.mean(y_coords) + slice_obj[0].start)
            cx = int(np.mean(x_coords) + slice_obj[1].start)
            cy, cx = self._clamp_to_free(cy, cx)
            region_centers[sl] = (cy, cx)
            path = self._astar(self.home_pixel, (cy, cx), self.free_mask)
            region_path_dists[sl] = len(path) if path else float('inf')
        rag = graph.RAG(labels, connectivity=2)

        def merge_cost(r1, r2):
            area1 = np.sum(labels == r1)
            area2 = np.sum(labels == r2)
            area_cost = (1.0 / area1) + (1.0 / area2)
            dist_cost = (region_path_dists[r1] + region_path_dists[r2])
            return self.alpha * area_cost + self.beta * dist_cost

        merged = True
        while merged:
            merged = False
            min_cost = float('inf')
            best_edge = None
            for (r1, r2) in rag.edges():
                cost = merge_cost(r1, r2)
                if cost < min_cost:
                    min_cost = cost
                    best_edge = (r1, r2)
            if best_edge and min_cost < self.merge_cost_threshold:
                r1, r2 = best_edge
                labels[labels == r2] = r1
                region_path_dists[r1] = min(
                    region_path_dists[r1], region_path_dists[r2])
                rag.merge_nodes(r2, r1)
                merged = True
        self.final_labels, _, _ = graph.relabel_sequential(labels)
        self.final_num_regions = np.max(self.final_labels)
        print(f"  -> Merged into {self.final_num_regions} final regions.")

    def step5_calculate_centers(self):
        """Compute reachable region centers (unsorted)."""
        print("Step 5: Calculating reachable centers...")
        self.reachable_centers = []
        regions_props = ndimage.find_objects(self.final_labels)
        for sl in range(1, self.final_num_regions + 1):
            slice_obj = regions_props[sl - 1]
            y_coords, x_coords = np.where(self.final_labels[slice_obj] == sl)
            cy = int(np.mean(y_coords) + slice_obj[0].start)
            cx = int(np.mean(x_coords) + slice_obj[1].start)
            path = self._astar(self.home_pixel, (cy, cx), self.free_mask)
            if path is not None:
                self.reachable_centers.append((cy, cx))
            else:
                direction_y = cy - self.home_pixel[0]
                direction_x = cx - self.home_pixel[1]
                length = math.sqrt(direction_y ** 2 + direction_x ** 2)
                if length == 0:
                    continue
                found = False
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
                            self.reachable_centers.append((test_y, test_x))
                            found = True
                            break
                if not found:
                    print(f"    -> Warning: Region {sl} center unreachable, "
                          f"skipped.")

    def _clamp_to_free(self, y, x):
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
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.free_mask.shape[0] - py) * self.resolution
        return (wx, wy)

    def process(self, home_pixel):
        self.step1_preprocess(home_pixel)
        self.step2_find_seeds(min_peak_distance=20)
        self.step3_voronoi_partition()
        self.step4_optimize_and_merge()
        self.step5_calculate_centers()
        return self.reachable_centers

    def save_results(self, output_img_path, output_txt_path):
        print(f"Saving results to {output_img_path} and {output_txt_path}...")
        vis_img = np.ones(
            (self.free_mask.shape[0], self.free_mask.shape[1], 3),
            dtype=np.uint8) * 255
        vis_img[self.occ_map == 1] = (0, 0, 0)
        if hasattr(self, 'unknown_mask'):
            vis_img[self.unknown_mask] = (128, 128, 128)
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
        for cy, cx in self.reachable_centers:
            cv2.drawMarker(vis_img, (cx, cy), (0, 0, 255),
                           markerType=cv2.MARKER_CROSS,
                           markerSize=15, thickness=2)
        cv2.imwrite(output_img_path, vis_img)
        with open(output_txt_path, 'w') as f:
            f.write("# Region Centers (World Coordinates: x y)\n")
            for cy, cx in self.reachable_centers:
                wx, wy = self.pixel_to_world(cy, cx)
                f.write(f"{wx:.3f} {wy:.3f}\n")
        print("Done!")


def create_living_room_map():
    """Procedurally generate a furnished apartment occupancy grid."""
    os.makedirs('sim', exist_ok=True)

    img = np.ones((800, 800), dtype=np.uint8) * 254
    wall_color = 0
    furniture_color = 0

    def draw_wall_with_door(img, pt1, pt2, door_start, door_end, thickness=8):
        cv2.line(img, pt1, door_start, wall_color, thickness)
        cv2.line(img, door_end, pt2, wall_color, thickness)

    # Outer walls
    cv2.rectangle(img, (0, 0), (799, 799), wall_color, 16)

    # Front door (bottom-left gap)
    cv2.rectangle(img, (120, 784), (220, 799), 254, -1)

    # Balcony door (top-right gap)
    cv2.rectangle(img, (450, 0), (650, 15), 254, -1)

    # Living/dining divider wall with passage
    cv2.line(img, (450, 15), (450, 280), wall_color, 8)
    cv2.line(img, (450, 360), (450, 500), wall_color, 8)

    # Living room furniture
    cv2.rectangle(img, (60, 30), (400, 65), furniture_color, -1)
    cv2.rectangle(img, (80, 250), (420, 285), furniture_color, -1)
    cv2.rectangle(img, (170, 140), (330, 210), furniture_color, -1)
    cv2.rectangle(img, (350, 140), (420, 200), furniture_color, -1)

    # Dining room furniture
    cv2.rectangle(img, (550, 150), (750, 320), furniture_color, -1)
    cv2.rectangle(img, (760, 100), (784, 300), furniture_color, -1)

    # Kitchen cabinets and fridge
    cv2.rectangle(img, (600, 500), (784, 530), furniture_color, -1)
    cv2.rectangle(img, (720, 400), (750, 500), furniture_color, -1)
    cv2.rectangle(img, (750, 600), (784, 700), furniture_color, -1)

    # Shoe cabinet (entryway)
    cv2.rectangle(img, (20, 600), (50, 750), furniture_color, -1)

    pgm_path = 'sim/sample_map.pgm'
    cv2.imwrite(pgm_path, img)
    print(f"PGM map saved to: {pgm_path}")

    yaml_content = (
        "image: sample_map.pgm\n"
        "resolution: 0.05\n"
        "origin: [-20.0, -20.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    yaml_path = 'sim/sample_map.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"YAML config saved to: {yaml_path}")


if __name__ == "__main__":
    print("=== Generating Living Room Map ===")
    create_living_room_map()

    yaml_file = 'sim/sample_map.yaml'
    partitioner = MapPartitioner(
        yaml_path=yaml_file,
        inflation_radius=5,
        min_area=600,
        alpha=1.0,
        beta=0.002,
        merge_cost_threshold=1.2,
    )

    home_py, home_px = 740, 170

    print("\n=== Running MapPartitioner ===")
    centers = partitioner.process(home_pixel=(home_py, home_px))

    partitioner.save_results(
        output_img_path="living_room_partitioned.png",
        output_txt_path="living_room_centers.txt",
    )
