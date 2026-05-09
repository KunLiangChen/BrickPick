import os
import yaml
import cv2
import numpy as np
import heapq
from scipy import ndimage
from skimage.segmentation import watershed
from skimage.segmentation import relabel_sequential
from skimage import graph
# from skimage import color
import math

class MapPartitioner:
    def __init__(self, yaml_path, inflation_radius=3, min_area=500, 
                 alpha=1.0, beta=0.002, merge_cost_threshold=1.5, max_region_radius= 1):
        self.yaml_path = yaml_path
        self.inflation_radius = inflation_radius
        self.min_area = min_area
        self.alpha = alpha
        self.beta = beta
        self.merge_cost_threshold = merge_cost_threshold
        self.max_region_radius = max_region_radius
        
        # 加载地图
        self._load_map()
        
    def _load_map(self):
        """读取 yaml 和 pgm 地图文件"""
        with open(self.yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        pgm_path = os.path.join(os.path.dirname(self.yaml_path), config['image'])
        self.resolution = config['resolution']
        self.origin = config['origin']
        
        # 读取图像 (灰度图)
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError(f"Cannot load map image: {pgm_path}")
            
        # ROS 标准地图约定: 0(黑)=占用, 254/255(白)=自由, 205(灰)=未知
        self.occ_map = np.zeros_like(img, dtype=np.uint8)
        self.occ_map[img < 250] = 1   # Occupied
        self.occ_map[img >= 250] = 0  # Free
        
        # # 未知区域单独标记为 2 (可选，本算法主要关注 Free)
        # self.unknown_mask = (img > 100) & (img < 200)
        
        print(f"Map loaded. Size: {self.occ_map.shape}, Resolution: {self.resolution}")

    def _astar(self, start, goal, binary_mask):
        """A* 寻路算法 (8连通)"""
        if not binary_mask[start[0], start[1]] or not binary_mask[goal[0], goal[1]]:
            return None
            
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
            
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        closed_set = set()
        
        # 8连通方向
        neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # 重建路径
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
                        
                    # 对角线移动代价 sqrt(2)
                    move_cost = 1.414 if (dy != 0 and dx != 0) else 1.0
                    tentative_g_score = g_score[current] + move_cost
                    
                    if tentative_g_score < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))
        return None # 无路径

    def visualize_occ_map(self, home_pixel=None, save_path="occ_map_visualization.png"):
        """可视化占用地图"""
        plt.figure(figsize=(10, 5))
        
        # 创建RGB图像
        vis_map = np.ones((self.occ_map.shape[0], self.occ_map.shape[1], 3), dtype=np.uint8) * 255
        vis_map[self.occ_map == 1] = [0, 0, 0]  # 黑色表示障碍物
        
        # 安全检查：防止传入的坐标越界导致绘图崩溃
        if home_pixel is not None:
            h, w = self.occ_map.shape
            if not (0 <= home_pixel[0] < h and 0 <= home_pixel[1] < w):
                print(f"⚠️ Warning: home_pixel {home_pixel} is out of map bounds {self.occ_map.shape}!")
                home_pixel = None  # 置空，后续不绘制标记
                
        # ================= 子图1：二值化RGB图 =================
        plt.subplot(121)
        plt.title(f"Occupancy Map (Binary)\n0=Free(White), 1=Occupied(Black)\nShape: {self.occ_map.shape}")
        plt.imshow(vis_map)
        if home_pixel is not None:
            # 注意：plt.scatter 的坐标顺序是 (x, y) -> (col, row) -> (home_pixel[1], home_pixel[0])
            plt.scatter([home_pixel[1]], [home_pixel[0]], c='red', s=120, marker='x', linewidths=2.5, label='Home')
            plt.legend(loc='upper right')
        plt.axis('off')
        
        # ================= 子图2：原始灰度值图 =================
        plt.subplot(122)
        plt.title(f"Raw occ_map Values\nResolution: {self.resolution}m")
        plt.imshow(self.occ_map, cmap='gray', vmin=0, vmax=1)
        if home_pixel is not None:
            plt.scatter([home_pixel[1]], [home_pixel[0]], c='red', s=120, marker='x', linewidths=2.5)
        plt.colorbar(label='Occupancy (0=Free, 1=Occupied)')
        plt.axis('off')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        print(f"✅ Occupancy map saved to {save_path}")
        plt.show()
        
        # ================= 打印统计信息 =================
        total_pixels = self.occ_map.size
        free_pixels = np.sum(self.occ_map == 0)
        occ_pixels = np.sum(self.occ_map == 1)
        print(f"\n=== Occupancy Map Statistics ===")
        print(f"Total pixels: {total_pixels}")
        print(f"Free space (0): {free_pixels} ({free_pixels/total_pixels*100:.2f}%)")
        print(f"Occupied (1): {occ_pixels} ({occ_pixels/total_pixels*100:.2f}%)")
        print(f"Map bounds: rows [0-{self.occ_map.shape[0]}], cols [0-{self.occ_map.shape[1]}]")
        
        if home_pixel is not None:
            hp_val = self.occ_map[home_pixel[0], home_pixel[1]]
            print(f"Home pixel {home_pixel} value: {hp_val} (0=Free, 1=Occupied)")

    def step1_preprocess(self, home_pixel):
        print("Step 1: Preprocessing map...")
        self.home_pixel = tuple(home_pixel)
        
        # 1. 提取障碍物 (原图中 occ_map == 1 的地方)
        occ_mask = (self.occ_map == 1).astype(np.uint8)
        
        # 2. 障碍物向外膨胀 (向外长)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.inflation_radius*2+1, self.inflation_radius*2+1))
        inflated_occ_mask = cv2.dilate(occ_mask, kernel, iterations=1)
        
        # 3. 膨胀后的障碍物区域变为0，剩下的才是安全的自由空间
        free_mask = (inflated_occ_mask == 0).astype(np.uint8)
        
        # 4. 连通域分析...
        labeled_array, num_features = ndimage.label(free_mask)
        
        if labeled_array[self.home_pixel] == 0:
            raise ValueError("Home point is not in the free/inflated space!")
            
        self.free_mask = (labeled_array == labeled_array[self.home_pixel]).astype(np.uint8)
        print(f"  -> Extracted reachable free space. Pixels: {np.sum(self.free_mask)}")

    def step2_find_seeds(self, phys_peak_distance=0.3):
        """
        第二步：潜在积木区域预测 (寻找种子点)
        
        """
        print("Step 2: Finding seed points...")
        
        # 安全检查：确保在 step1 中已经读取了地图分辨率
        if not hasattr(self, 'resolution') or self.resolution <= 0:
            raise ValueError("Map resolution not found. Did step1 run correctly?")
            
        
        min_dist_pixels = int(phys_peak_distance / self.resolution)
        
        # 距离变换 (到障碍物的距离)
        dist_map = ndimage.distance_transform_edt(self.free_mask)
        self.dist_map = dist_map
        
        
        from skimage.feature import peak_local_max
        peaks = peak_local_max(
            dist_map, 
            min_distance=min_dist_pixels, 
            labels=self.free_mask,
            exclude_border=False # 允许在地图边缘出现种子
        )
        
        
        
        self.seeds = peaks
        print(f"  -> Physical min distance: {phys_peak_distance}m ({min_dist_pixels} px)")
        print(f"  -> Found {len(self.seeds)} core seed points.")


    def step3_voronoi_partition(self):
        """第三步：初步区域生成 (Voronoi分区 - 使用 Watershed 算法)"""
        print("Step 3: Generating Voronoi partitions...")
        if len(self.seeds) == 0:
            raise ValueError("No seed points found!")
            
        # 构造 Marker 图像
        markers = np.zeros(self.free_mask.shape, dtype=np.int32)
        for i, (y, x) in enumerate(self.seeds):
            markers[y, x] = i + 1 # 标签从 1 开始
            
        # 使用距离变换的负值作为 "海拔"，Watershed 会将水填入局部极小值
        # 这样自然形成了以 seeds 为中心的 Voronoi 图
        self.labels = watershed(-self.dist_map, markers, mask=self.free_mask)
        self.num_regions_init = np.max(self.labels)
        print(f"  -> Generated {self.num_regions_init} initial regions.")

    def step4_optimize_and_merge(self):
        """第四步：区域优化与合并 (带最大半径约束)"""
        print("Step 4: Optimizing and merging regions...")
        labels = self.labels
        
        # 1. 合并小区域 (代码保持不变，省略...)
        unique_labels, counts = np.unique(labels, return_counts=True)
        label_area_map = dict(zip(unique_labels, counts))
        small_labels = [l for l, area in label_area_map.items() if area < self.min_area and l != 0]
        for sl in small_labels:
            dilated = ndimage.binary_dilation(labels == sl)
            neighbors = np.unique(labels[dilated])
            neighbors = [n for n in neighbors if n != 0 and n != sl]
            if neighbors:
                max_neighbor = max(neighbors, key=lambda n: label_area_map.get(n, 0))
                labels[labels == sl] = max_neighbor
                label_area_map[max_neighbor] += label_area_map.get(sl, 0)
                if sl in label_area_map: del label_area_map[sl]
                
        labels, _, _ = relabel_sequential(labels)
        regions_props = ndimage.find_objects(labels)
        region_centers = {}
        region_path_dists = {}
        
        # 【新增】用于存储每个区域当前的最大半径 (像素)
        region_max_radii_px = {}
        
        print("  -> Calculating path distances and MAX RADII to region centers...")
        num_regions = np.max(labels)
        
        # 将极限半径转换为像素，用于后续比较
        max_radius_px_limit = self.max_region_radius / self.resolution
        
        for sl in range(1, num_regions + 1):
            slice_obj = regions_props[sl-1]
            if slice_obj is None: continue
            y_coords, x_coords = np.where(labels[slice_obj] == sl)
            if len(y_coords) == 0: continue
            
            cy = int(np.mean(y_coords) + slice_obj[0].start)
            cx = int(np.mean(x_coords) + slice_obj[1].start)
            cy, cx = self._clamp_to_free(cy, cx)
            region_centers[sl] = (cy, cx)
            
            # A* 计算真实路径距离
            path = self._astar(self.home_pixel, (cy, cx), self.free_mask)
            region_path_dists[sl] = len(path) if path else float('inf')
            
            # 【关键新增】计算该区域内所有像素到中心点的最大欧式距离
            dists_to_center = np.sqrt((y_coords - cy)**2 + (x_coords - cx)**2)
            region_max_radii_px[sl] = np.max(dists_to_center) if len(dists_to_center) > 0 else 0
            
        # Greedy Merge Loop
        merged = True
        while merged:
            merged = False
            min_cost = float('inf')
            best_edge = None
            
            rag = graph.RAG(labels, connectivity=2)
            
            def merge_cost(r1, r2):
                area1 = np.sum(labels == r1)
                area2 = np.sum(labels == r2)
                if area1 == 0 or area2 == 0: return float('inf')
                
                # 【关键新增】最大半径硬约束检查
                c1 = region_centers.get(r1)
                c2 = region_centers.get(r2)
                if c1 and c2:
                    # 两个旧中心点之间的距离
                    center_dist = math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2)
                    # 极其保守地预估合并后的最大半径：两中心距 + 两者中更大的原始半径
                    # (实际合并后中心会偏移，最大半径通常小于此值，所以这是绝对安全的上界)
                    estimated_merged_radius = center_dist + max(region_max_radii_px.get(r1, 0), region_max_radii_px.get(r2, 0))
                    
                    if estimated_merged_radius > max_radius_px_limit:
                        return float('inf') # 一票否决：合并后必然超限，绝对不允许合并！

                area_cost = (1.0 / area1) + (1.0 / area2) 
                dist_cost = region_path_dists.get(r1, float('inf')) + region_path_dists.get(r2, float('inf'))
                return self.alpha * area_cost + self.beta * dist_cost

            for (r1, r2) in rag.edges():
                if r1 == 0 or r2 == 0: continue
                cost = merge_cost(r1, r2)
                if cost < min_cost:
                    min_cost = cost
                    best_edge = (r1, r2)
                    
            if best_edge and min_cost < self.merge_cost_threshold:
                r1, r2 = best_edge
                labels[labels == r2] = r1
                
                # 更新路径距离
                d1 = region_path_dists.get(r1, float('inf'))
                d2 = region_path_dists.get(r2, float('inf'))
                region_path_dists[r1] = min(d1, d2)
                if r2 in region_path_dists: del region_path_dists[r2]
                
                # 【关键新增】更新合并后新区域的真实最大半径
                y_new, x_new = np.where(labels == r1)
                if len(y_new) > 0:
                    c_new = region_centers[r1]
                    new_dists = np.sqrt((y_new - c_new[0])**2 + (x_new - c_new[1])**2)
                    region_max_radii_px[r1] = np.max(new_dists)
                if r2 in region_max_radii_px: del region_max_radii_px[r2]
                
                merged = True
                
        self.final_labels, _, _ = relabel_sequential(labels)
        self.final_num_regions = np.max(self.final_labels)
        print(f"  -> Merged into {self.final_num_regions} final regions (Respecting {self.max_region_radius}m radius limit).")
    
    def step5_calculate_centers(self):
        """第五步：计算可达中心点"""
        print("Step 5: Calculating reachable centers...")
        self.reachable_centers = []
        regions_props = ndimage.find_objects(self.final_labels)
        
        for sl in range(1, self.final_num_regions + 1):
            slice_obj = regions_props[sl-1]
            y_coords, x_coords = np.where(self.final_labels[slice_obj] == sl)
            
            # 1. 计算几何中心 (质心)
            cy = int(np.mean(y_coords) + slice_obj[0].start)
            cx = int(np.mean(x_coords) + slice_obj[1].start)
            
            # 2. 可达性验证
            path = self._astar(self.home_pixel, (cy, cx), self.free_mask)
            if path is not None:
                self.reachable_centers.append((cy, cx))
            else:
                # 3. 修正：沿 Home 到中心点的方向线回退搜索
                direction_y = cy - self.home_pixel[0]
                direction_x = cx - self.home_pixel[1]
                length = math.sqrt(direction_y**2 + direction_x**2)
                if length == 0: continue
                
                found = False
                for step in np.linspace(0, length, num=int(length)):
                    test_y = int(self.home_pixel[0] + (direction_y / length) * step)
                    test_x = int(self.home_pixel[1] + (direction_x / length) * step)
                    test_y, test_x = self._clamp_to_free(test_y, test_x)
                    
                    # 确保测试点仍然在当前区域内
                    if self.final_labels[test_y, test_x] == sl:
                        test_path = self._astar(self.home_pixel, (test_y, test_x), self.free_mask)
                        if test_path is not None:
                            self.reachable_centers.append((test_y, test_x))
                            found = True
                            break
                if not found:
                    print(f"    -> Warning: Region {sl} center unreachable, skipped.")

    def _clamp_to_free(self, y, x):
        """将越界或落在障碍物上的点修正到最近的自由空间点"""
        h, w = self.free_mask.shape
        y, x = np.clip(y, 0, h-1), np.clip(x, 0, w-1)
        if self.free_mask[y, x] == 0:
            # 简单的局部搜索寻找最近自由点
            for r in range(1, 10):
                for dy in range(-r, r+1):
                    for dx in range(-r, r+1):
                        ny, nx = y+dy, x+dx
                        if 0 <= ny < h and 0 <= nx < w and self.free_mask[ny, nx] == 1:
                            return ny, nx
        return y, x

    def pixel_to_world(self, py, px):
        """将像素坐标转换为世界坐标"""
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.free_mask.shape[0] - py) * self.resolution # ROS Y轴向上
        return (wx, wy)

    def process(self, home_pixel):
        """执行完整流水线"""
        self.visualize_occ_map(home_pixel)
        self.step1_preprocess(home_pixel)
        self.step2_find_seeds()
        self.step3_voronoi_partition()
        self.step4_optimize_and_merge()
        self.step5_calculate_centers()
        return self.reachable_centers

    def save_results(self, output_img_path, output_txt_path):
        """输出可视化结果与中心点文件"""
        print(f"Saving results to {output_img_path} and {output_txt_path}...")
        
        # 1. 可视化
        # 创建彩色背景：黑(占用)，灰(未知)，白(自由)
        vis_img = np.ones((self.free_mask.shape[0], self.free_mask.shape[1], 3), dtype=np.uint8) * 255
        vis_img[self.occ_map == 1] = (0, 0, 0)
        # vis_img[self.unknown_mask] = (128, 128, 128)
        
        # 给分区上色
        # 生成随机但高对比度的颜色
        np.random.seed(42)
        colors = np.random.randint(50, 255, size=(self.final_num_regions + 1, 3))
        colors[0] = [255, 255, 255] # 背景白
        
        for sl in range(1, self.final_num_regions + 1):
            vis_img[self.final_labels == sl] = colors[sl]
            
        # 画出种子点 (蓝色)
        for y, x in self.seeds:
            cv2.circle(vis_img, (x, y), 2, (255, 0, 0), -1)
            
        # 画 Home 点 (绿色)
        cv2.circle(vis_img, (self.home_pixel[1], self.home_pixel[0]), 6, (0, 255, 0), -1)
        
        # 画可达中心点 (红色，带十字)
        for cy, cx in self.reachable_centers:
            cv2.drawMarker(vis_img, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
            
        cv2.imwrite(output_img_path, vis_img)
        
        # 2. 保存坐标 (世界坐标系)
        with open(output_txt_path, 'w') as f:
            f.write("# Region Centers (World Coordinates: x y)\n")
            for cy, cx in self.reachable_centers:
                wx, wy = self.pixel_to_world(cy, cx)
                f.write(f"{wx:.3f} {wy:.3f}\n")
                
        print("Done!")

    def world_to_pixel(self, wx, wy):
        # 注意：此函数需要在 _load_map 之后调用，因为需要图像高度
        dx = wx - self.origin[0]
        dy = wy - self.origin[1]
        px = int(dx / self.resolution)
        py = int(self.occ_map.shape[0] - (dy / self.resolution))
        return (py, px)

import matplotlib.pyplot as plt

def visualize_step2_details(self, phys_peak_distance=0.6):
    """
    专门可视化 Step 2 的内部逻辑：距离变换与峰值检测
    """
    print("Visualizing Step 2 details...")
    
    # 重新计算（确保数据最新）
    min_dist_pixels = int(phys_peak_distance / self.resolution)
    dist_map = ndimage.distance_transform_edt(self.free_mask)
    
    from skimage.feature import peak_local_max
    peaks = peak_local_max(
        dist_map, 
        min_distance=min_dist_pixels, 
        labels=self.free_mask,
        exclude_border=False
    )

    # 开始绘图
    plt.figure(figsize=(18, 5))

    # 子图1：原始自由空间 (Binary Mask)
    plt.subplot(131)
    plt.title(f"1. Reachable Free Space\n(Input for Step 2)")
    plt.imshow(self.free_mask, cmap='gray')
    plt.axis('off')

    # 子图2：距离变换图 (Distance Transform)
    # 颜色越亮表示离墙越远，就像地形图上的“山峰”
    
    plt.subplot(132)
    plt.title(f"2. Distance Transform\n(The 'Mountain' Map)")
    im2 = plt.imshow(dist_map, cmap='viridis')
    plt.colorbar(im2, fraction=0.046, pad=0.04)
    plt.axis('off')

    # 子图3：检测到的峰值 (Seeds)
    plt.subplot(133)
    plt.title(f"3. Local Maxima Extracted\n(Seeds, dist={phys_peak_distance}m)")
    plt.imshow(dist_map, cmap='magma')
    # 画出种子点，注意坐标是 (row, col) 对应 (y, x)
    plt.plot(peaks[:, 1], peaks[:, 0], 'r.', markersize=8, label='Seeds')
    plt.legend()
    plt.axis('off')

    plt.tight_layout()
    plt.show()



# 在主程序中调用
# partitioner.step1_preprocess(home_pixel=(80, 80))
# visualize_step2_details(partitioner, phys_peak_distance=0.6)

# ================= 测试用例 =================
if __name__ == "__main__":
    # 假设你有一个ROS格式的地图文件
    # 请将 'my_map.yaml' 替换为你的实际文件路径
    
    yaml_file = 'map/livingroom_clean_12.yaml'
    
    # 初始化分区器 (参数可调)
    # inflation_radius: 机器人安全半径/栅格分辨率
    # min_area: 小于此像素面积的区域将被强制合并
    # merge_cost_threshold: 控制最终区域数量的阈值，越小区域越多，越大区域越少
    partitioner = MapPartitioner(
        yaml_path=yaml_file, 
        inflation_radius=3, 
        min_area=300,
        alpha=1.0, 
        beta=0.002, 
        merge_cost_threshold=2.0
    )
    # home_world_x = partitioner.origin[0]
    # home_world_y = partitioner.origin[1]
    # 设置 Home 点 (像素坐标: 行, 列)
    # 通常在地图底部中间找一块空地
    # home_py, home_px = partitioner.world_to_pixel(home_world_x + 0.5, home_world_y + 0.5)
    home_py, home_px = 200, 78 

    print(f"Origin from YAML: {partitioner.origin[:2]}")
    print(f"Calculated Home Pixel: {home_py, home_px}")
    
    # 运行算法
    centers = partitioner.process(home_pixel=(home_py, home_px))
    
    # 保存结果
    partitioner.save_results(
        output_img_path="partitioned_map.png", 
        output_txt_path="region_centers.txt"
    )