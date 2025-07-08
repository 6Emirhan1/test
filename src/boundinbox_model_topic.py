import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json
import time

class YoloDepthPublisher(Node):
    def __init__(self):
        super().__init__('yolo_depth_publisher')
        self.bridge = CvBridge()
        self.model = YOLO("/media/tezcan/.../fine_tuning_best.pt")  # Model yolunu düzelt
        self.get_logger().info("YOLO modeli yüklendi (conf >= 0.7).")

        self.latest_depth = None
        self.last_published_sign = None
        self.last_publish_time = 0
        self.time_threshold = 8.0  # 8 saniyelik zaman eşiği

        # Abonelikler
        self.create_subscription(Image,
                                 '/carla/hero/depth_front/image',
                                 self.depth_cb, 10)
        self.create_subscription(Image,
                                 '/carla/hero/rgb_front/image',
                                 self.rgb_cb, 10)

        # Yayıncı: sadece levha adlarını
        self.pub = self.create_publisher(String,
                                         '/astrid/perception/traffic_signs',
                                         10)

        # Carla RGB çözünürlüğü (sabit)
        self.rgb_w = 1024
        self.rgb_h = 768

    def depth_cb(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.float32)
            self.latest_depth = arr.reshape((msg.height, msg.width))
        except Exception as e:
            self.get_logger().error(f"Depth okunamadı: {e}")
            self.latest_depth = None

    def rgb_cb(self, msg: Image):
        if self.latest_depth is None:
            return

        current_time = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, device='cpu', imgsz=640, conf=0.7)

        # Görselleştirme
        vis = results[0].plot()
        cv2.imshow("YOLO Detection", vis)
        cv2.waitKey(1)

        dh, dw = self.latest_depth.shape
        sx, sy = dw / self.rgb_w, dh / self.rgb_h

        signs_to_publish = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            name = self.model.names[cls_id]
            x_c, y_c = box.xywh[0][:2].cpu().numpy()

            xd = int(x_c * sx)
            yd = int(y_c * sy)

            if not (0 <= xd < dw and 0 <= yd < dh):
                continue

            dist = float(self.latest_depth[yd, xd])
            sign_distance_threshold = 12.0
            
            # Mesafe kontrolü
            if not np.isfinite(dist) or dist > sign_distance_threshold:
                continue
            
            self.get_logger().info(f"Tespit edilen levha: {name}, Mesafe: {dist:.2f}m")
            
            # Zaman kontrolü (sadece aynı levha için)
            if name == self.last_published_sign and (current_time - self.last_publish_time) < self.time_threshold:
                continue
            
            signs_to_publish.append(name)

        if signs_to_publish:
            unique_signs = list(set(signs_to_publish))
            msg_out = String()
            msg_out.data = json.dumps(unique_signs)
            self.pub.publish(msg_out)
            self.last_publish_time = current_time
            self.last_published_sign = unique_signs[0]  # İlk yayınlanan işareti kaydet
            self.get_logger().info(f"Yayınlanan levha(lar): {unique_signs}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()