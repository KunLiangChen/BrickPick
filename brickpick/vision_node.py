#!/usr/bin/env python3
"""
brickpick: 视觉识别节点
- 订阅相机图像话题
- 使用 YOLO 进行物体检测
- 发布标注后的图像和检测结果 (Bounding Box)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class BrickVisionNode(Node):
    def __init__(self):
        super().__init__('brickpick_vision')
        
        # 1. 声明与读取参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', 'yolo11n.pt'),
                ('camera_topic', 'camera/image_color'),
                ('conf_threshold', 0.25),
                ('iou_threshold', 0.45),
                ('imgsz', 640),
                ('publish_debug_image', True),
                ('device', 'cpu') # 'cpu' or '0' (cuda)
            ]
        )
        
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.imgsz = self.get_parameter('imgsz').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.device = self.get_parameter('device').value
        
        # 2. 初始化 YOLO 和 CvBridge
        self.get_logger().info(f"正在加载 YOLO 模型: {model_path} 在设备 {self.device} 上...")
        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            raise e
            
        self.bridge = CvBridge()
        
        # 3. 订阅与发布
        camera_topic = self.get_parameter('camera_topic').value
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
            
        # 结构化检测结果发布
        self.detection_pub = self.create_publisher(
            Detection2DArray, 
            'vision/detections', 
            10)
            
        # 调试画面发布
        if self.publish_debug:
            self.image_pub = self.create_publisher(
                Image, 
                'vision/annotated_image', 
                10)

        self.get_logger().info(f"Vision Node 启动成功，监听话题: {camera_topic}")

    def image_callback(self, msg):
        try:
            # 1. 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. YOLO 推理
            results = self.model.predict(
                source=cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False
            )
            
            # 3. 构建 Detection2DArray 消息
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # 边界框坐标 [x1, y1, x2, y2]
                    # vision_msgs 使用 center.x, center.y, size_x, size_y
                    xywh = box.xywh[0].cpu().numpy()
                    detection.bbox.center.position.x = float(xywh[0])
                    detection.bbox.center.position.y = float(xywh[1])
                    detection.bbox.size_x = float(xywh[2])
                    detection.bbox.size_y = float(xywh[3])
                    
                    # 类别与置信度
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(int(box.cls[0]))
                    hyp.hypothesis.score = float(box.conf[0])
                    detection.results.append(hyp)
                    
                    detection_array.detections.append(detection)
            
            # 4. 发布结果
            self.detection_pub.publish(detection_array)
            
            # 5. 可视化调试
            if self.publish_debug:
                annotated_frame = results[0].plot()
                out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                out_msg.header = msg.header
                self.image_pub.publish(out_msg)
                
        except Exception as e:
            self.get_logger().error(f"处理图像时发生错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BrickVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
