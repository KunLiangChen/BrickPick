#!/usr/bin/env python3
import numpy as np
import cv2
import yaml
import os
import matplotlib.pyplot as plt
from pathlib import Path

class CircularMapDecomposer:
    def __init__(self, yaml_path: str, max_radius: float, overlap_ratio: float = 0.0, min_free_ratio: float = 0.6):
        self.max_radius = max_radius
        self.overlap_ratio = overlap_ratio
        self.min_free_ratio = min_free_ratio
        self.yaml_path = yaml_path
        self.pgm_path = yaml_path.replace('.yaml', '.pgm')
        
        # 加载地图元数据与图像
        self._load_map()
        
    def _load_map(self):
        with open(self.yaml_path, 'r') as f:
            self.meta = yaml.safe_load(f)
            
        # 读取 pgm (0=occupied/black, 255=free/white, 205=unknown)
        img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Cannot load {self.pgm_path}")
            
        # ROS 阈值过滤: free_thresh 通常为 0.196, occupied_thresh 为 0.65
        free_thresh = self.meta.get('free_thresh', 0.196) * 255.0
        # 生成自由空间二值掩码 (True=free, False=occupied/unknown)
        self.free_mask = img > free_thresh
        self.height, self.width = self.free_mask.shape
        self.resolution = self.meta['resolution']
        self.origin = self.meta['origin'][:2]  # [x, y]

    def decompose(self) -> list[tuple[float, float]]:
        r_px = int(np.ceil(self.max_radius / self.resolution))
        stride = int(2 * self.max_radius / self.resolution * (1 - self.overlap_ratio))
        stride = max(1, stride)
        
        # 预计算圆形二值核 (用于快速计算圆内自由像素比例)
        y, x = np.ogrid[-r_px:r_px+1, -r_px:r_px+1]
        circle_kernel = (x**2 + y**2) <= r_px**2
        
        centers = []
        # 滑动网格采样
        for r in range(r_px, self.height - r_px, stride):
            for c in range(r_px, self.width - r_px, stride):
                r_s, r_e = r - r_px, r + r_px + 1
                c_s, c_e = c - r_px, c + r_px + 1
                
                # 提取对应区域
                region = self.free_mask[r_s:r_e, c_s:c_e]
                # 计算圆内自由像素比例
                free_ratio = np.sum(region & circle_kernel) / np.sum(circle_kernel)
                
                if free_ratio >= self.min_free_ratio:
                    # 像素坐标 -> 世界坐标 (注意 ROS 地图 Y 轴翻转)
                    x_w = self.origin[0] + (c + 0.5) * self.resolution
                    y_w = self.origin[1] + (self.height - r - 0.5) * self.resolution
                    centers.append((round(x_w, 3), round(y_w, 3)))
                    
        return centers

    def visualize(self, centers):
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.imshow(self.free_mask, cmap='gray', origin='upper')
        ax.set_title(f"Sub-region Centers (N={len(centers)})")
        
        r_px = self.max_radius / self.resolution
        for x_w, y_w in centers:
            # 世界坐标转像素坐标用于绘制
            c = int((x_w - self.origin[0]) / self.resolution - 0.5)
            r = int(self.height - (y_w - self.origin[1]) / self.resolution - 0.5)
            circle = plt.Circle((c, r), r_px, color='r', fill=False, linewidth=1)
            ax.add_patch(circle)
            ax.plot(c, r, 'b.', markersize=4)
            
        ax.axis('off')
        plt.tight_layout()
        out_img = Path(self.yaml_path).parent / "decomposition_result.png"
        plt.savefig(out_img, dpi=150)
        print(f"可视化已保存至: {out_img}")
        plt.show()


if __name__ == "__main__":
    # ================= 配置区 =================
    MAP_YAML = "my_map.yaml"          # 替换为你的实际路径
    MAX_RADIUS = 5.0                  # 最大半径 (米)
    OVERLAP_RATIO = 0.1               # 重叠比例 (0~0.3)，推荐 0.1 保障边界连续性
    MIN_FREE_RATIO = 0.6              # 圆内自由空间占比阈值
    # ==========================================
    
    if not os.path.exists(MAP_YAML):
        print(f"❌ 找不到地图文件: {MAP_YAML}")
        print("请将脚本放在与 my_map.yaml / my_map.pgm 同级目录，或修改 MAP_YAML 路径")
        exit(1)

    decomposer = CircularMapDecomposer(MAP_YAML, MAX_RADIUS, OVERLAP_RATIO, MIN_FREE_RATIO)
    centers = decomposer.decompose()
    
    print(f"📍 共生成 {len(centers)} 个子区域中心点 (单位: 米):")
    for i, (x, y) in enumerate(centers, 1):
        print(f"  {i:3d}: ({x:8.3f}, {y:8.3f})")
        
    # 可选：保存到 CSV 供其他节点读取
    import csv
    with open("subregion_centers.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x_m", "y_m"])
        for i, (x, y) in enumerate(centers, 1):
            writer.writerow([i, x, y])
    print("中心点已导出至 subregion_centers.csv")
    
    decomposer.visualize(centers)