
import cv2
import torch
from ultralytics import YOLO
from .. import config

class BlockInfo:
    def __init__(self, x1, y1, x2, y2, cls, conf, frame_width, frame_height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.cls = cls
        self.conf = conf
        self.center_x = (x1 + x2) / 2
        self.center_y = (y1 + y2) / 2
        self.width = x2 - x1
        self.height = y2 - y1
        self.area = self.width * self.height
        self.frame_center_x = frame_width / 2
        self.frame_center_y = frame_height / 2
        self.distance_to_center = ((self.center_x - self.frame_center_x) ** 2 +
                                   (self.center_y - self.frame_center_y) ** 2) ** 0.5

    def __str__(self):
        return f"Block(center:({self.center_x:.1f},{self.center_y:.1f}), area:{self.area:.1f}, dist:{self.distance_to_center:.1f})"

class Detector:
    def __init__(self, model_path=config.MODEL_PATH, device='cuda' if torch.cuda.is_available() else 'cpu'):
        try:
            self.model = YOLO(model_path)
            self.device = device
            self.model.to(self.device)
            print(f"YOLO model loaded on {self.device}")
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            raise

    def detect_blocks(self, img):
        if img is None:
            return []

        try:
            results = self.model(img, conf=config.CONFIDENCE_THRESHOLD, device=self.device,
                                 imgsz=config.YOLO_IMG_SIZE, verbose=False)
        except Exception as e:
            print(f"YOLO detection error: {e}")
            return []

        blocks = []
        if results and len(results) > 0:
            result = results[0]
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for i in range(len(boxes)):
                    box = boxes.xyxy[i].cpu().numpy()
                    cls = int(boxes.cls[i].cpu().numpy())
                    conf = float(boxes.conf[i].cpu().numpy())

                    if (cls == 0 or cls == 1) and conf >= config.CONFIDENCE_THRESHOLD:
                        block = BlockInfo(box[0], box[1], box[2], box[3], cls, conf,
                                          img.shape[1], img.shape[0])
                        blocks.append(block)
        return blocks

def select_best_block(blocks, frame_width, frame_height):
    """选择最佳的积木目标"""
    if not blocks:
        return None

    # 根据策略选择新目标
    if config.TARGET_SELECTION_STRATEGY == "closest":
        # 选择距离画面中心最近的目标
        best_block = min(blocks, key=lambda b: b.distance_to_center)
    elif config.TARGET_SELECTION_STRATEGY == "largest":
        # 选择面积最大的目标
        best_block = max(blocks, key=lambda b: b.area)
    elif config.TARGET_SELECTION_STRATEGY == "center":
        # 选择最靠近画面中心的目标（考虑画面尺寸）
        center_x, center_y = frame_width / 2, frame_height / 2
        best_block = min(blocks, key=lambda b: abs(b.center_x - center_x) + abs(b.center_y - center_y))
    else:
        best_block = blocks[0]

    print(f"选择新目标: {best_block}")
    return best_block
