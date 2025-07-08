import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import numpy as np
import time

class TrafficSignIDPublisher(Node):
    def __init__(self):
        super().__init__('traffic_sign_id_publisher')
        self.latest_depth = None
        self.last_published_sign = None
        self.last_publish_time = 0
        self.time_threshold = 7.0  # 7 saniyelik zaman eşiği

        # Abonelikler
        self.sign_sub = self.create_subscription(
            String,
            '/astrid/perception/traffic_sign',
            self.sign_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/carla/hero/depth_front/image',
            self.depth_callback,
            10)

        # Yayıncı
        self.id_pub = self.create_publisher(String, '/astrid/perception/traffic_signs', 10)
        self.get_logger().info("Trafik levhası ID yayınlayıcı başlatıldı.")

    def depth_callback(self, msg: Image):
        try:
            depth_array = np.frombuffer(msg.data, dtype=np.float32)
            self.latest_depth = depth_array.reshape((msg.height, msg.width))
        except Exception as e:
            self.get_logger().error(f"Depth verisi okunamadı: {e}")
            self.latest_depth = None

    def sign_callback(self, msg: String):
        if self.latest_depth is None:
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Geçersiz JSON formatı.")
            return

        current_time = time.time()
        height, width = self.latest_depth.shape
        signs_to_publish = []

        rgb_width = 1024
        rgb_height = 768
        scale_x = width / rgb_width
        scale_y = height / rgb_height

        for det in detections:
            confidence = det.get("confidence", 0.0)
            if confidence < 0.7:
                continue

            class_name = det["class_name"]
            x_depth = int(det["x"] * scale_x)
            y_depth = int(det["y"] * scale_y)

            if 0 <= x_depth < width and 0 <= y_depth < height:
                distance = float(self.latest_depth[y_depth, x_depth])

                if np.isfinite(distance) and distance > 0.01 and distance <= 12.0:
                    # Zaman kontrolü (sadece aynı levha için)
                    if class_name == self.last_published_sign and (current_time - self.last_publish_time) < self.time_threshold:
                        continue  # Aynı levhayı 5s içinde tekrar yayınlama
                    signs_to_publish.append(class_name)

        if signs_to_publish:
            unique_signs = list(set(signs_to_publish))
            msg_out = String()
            msg_out.data = json.dumps(unique_signs)
            self.id_pub.publish(msg_out)
            self.last_publish_time = current_time
            self.last_published_sign = unique_signs[0] if unique_signs else None
            self.get_logger().info(f"Yayınlanan levha(lar): {unique_signs} ve tespit mesafesi {distance}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignIDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()