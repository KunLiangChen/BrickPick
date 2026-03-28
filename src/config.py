# --- 配置 ---
# YOLOv8 模型路径
MODEL_PATH = "./model/best.pt"  # <--- 请修改为你的模型文件路径
CONFIDENCE_THRESHOLD = 0.5  # 检测置信度阈值
ALIGNMENT_TOLERANCE = 50  # 对准容差 (像素)
MOVE_SPEED = 0.25  # 前进速度 (m/s)
ROTATE_SPEED_P = 0.05  # 旋转速度比例系数
DISTANCE_THRESHOLD = 270  # 抓取距离阈值 (cm)
DISTANCE_SENSOR_INDEX = 3  # 测距仪连接的传感器板索引 (根据实际情况调整)

# Marker相关配置
TARGET_MARKER_NAME = "heart"  # 指定要识别的 Marker 名称
MARKER_DISTANCE_THRESHOLD = 0.20  # Marker距离阈值

# 搜索相关配置
SEARCH_ROTATION_SPEED = 15  # 搜索旋转速度
SEARCH_TIMEOUT = 30  # 搜索超时时间(秒)
ALIGNMENT_STABILITY_FRAMES = 5  # 对齐稳定帧数（增加稳定性）
MISSING_DETECTION_THRESHOLD = 10  # 丢失检测阈值
GRAB_CONFIRMATION_TIME = 1  # 抓取确认时间
BACKWARD_DISTANCE = 0.4  # 后退距离(米)

# 多目标处理配置
TARGET_SELECTION_STRATEGY = "closest"  # 目标选择策略: "closest", "largest", "center"
STICKY_TARGET_FRAMES = 10  # 粘性目标帧数，保持跟踪同一目标

# 性能优化配置
PROCESS_EVERY_N_FRAMES = 3  # 每N帧处理一次推理
YOLO_IMG_SIZE = 640  # YOLO输入图像尺寸
