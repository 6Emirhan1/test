#!/usr/bin/env python3
import json
import lanelet2
import lanelet2.io
import lanelet2.projection
import lanelet2.traffic_rules
import lanelet2.routing
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32, String
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from shapely.geometry import Point, Polygon

# Harita Yükleme
filename = "/home/emirhan/Documents/simulation_fulltrackv6.osm"
latitude, longitude = 0.0, 0.0
origin = lanelet2.io.Origin(latitude, longitude)
projector = lanelet2.projection.LocalCartesianProjector(origin)

map = lanelet2.io.load(filename, projector)
traffic_rules = lanelet2.traffic_rules.create(
    lanelet2.traffic_rules.Locations.Germany,
    lanelet2.traffic_rules.Participants.Vehicle,
)
routing_graph = lanelet2.routing.RoutingGraph(map, traffic_rules)

print("Harita başarıyla yüklendi!")
print("Lanelet sayısı:", len(map.laneletLayer))

# Yardımcı Fonksiyonlar --------------------------------------------------------
def get_local_coords(point):
    """.osm dosyasındaki noktadan local_x, local_y koordinatlarını alır"""
    x_tag = point.attributes['local_x']
    y_tag = point.attributes['local_y']
    if x_tag is None or y_tag is None:
        return None
    return float(x_tag), float(y_tag)

def relevant_direction_exists(sign, followers):
    directions = []
    for f in followers:
        if "turn_direction" in f.attributes:
            td = f.attributes["turn_direction"]
            directions.append(td)

    if sign == "saga donulmez":
        return "right" in directions
    elif sign == "sola donulmez":
        return "left" in directions
    elif sign == "sola mecburi yon":
        return "left" in directions
    elif sign == "saga mecburi yon":
        return "right" in directions
    elif sign == "girilmez":
        return "straight" in directions
    elif sign == "Durak":
        return True
    elif sign == "Park yeri":
        return True
    elif sign == "Ileri mecburi yon":
        return "straight" in directions
    elif sign == "Ileri ve saga mecburi yon":
        return "straight" in directions or "right" in directions
    elif sign == "Ileri ve sola mecburi yon":
        return "straight" in directions or "left" in directions
    elif sign in ["Ileriden sola mecburi yon", "Ileriden saga mecburi yon"]:
        return any(
            "turn_direction" in f.attributes and f.attributes["turn_direction"] == "straight"
            for f in followers
        )
    return False

def straight_followed_but_turn_missing(followers, required_td):
    for nxt in followers:
        td = nxt.attributes["turn_direction"] if "turn_direction" in nxt.attributes else ""
        if td == "straight":
            second_followers = routing_graph.following(nxt)
            for sec in second_followers:
                sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else ""
                if sec_td == required_td:
                    return False
            return True
    return False

# ROS Düğümleri --------------------------------------------------------------
class LocalLaneletMatcher(Node):
    def __init__(self):
        super().__init__('local_lanelet_matcher')
        self.subscription = self.create_subscription(
            Odometry, '/carla/hero/odometry', self.odom_callback, 10
        )
        self.pub = self.create_publisher(Int32, '/astrid/slam/current_lanelet_id', 10)
        self.last_lanelet_id = None

    def find_current_lanelet(self, x, y):
        pt = Point(x, y)
        for ll in map.laneletLayer:
            left = []
            for p in ll.leftBound:
                coord = get_local_coords(p)
                if coord:
                    left.append(coord)
            right = []
            for p in reversed(ll.rightBound):
                coord = get_local_coords(p)
                if coord:
                    right.append(coord)
            coords = left + right
            if len(coords) < 3:
                continue
            if coords[0] != coords[-1]:
                coords.append(coords[0])
            poly = Polygon(coords)
            if not poly.is_valid:
                continue
            if poly.covers(pt):
                return ll.id
        return None

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_id = self.find_current_lanelet(x, y)
        
        if current_id != self.last_lanelet_id:
            if current_id is not None:
                self.get_logger().info(f" Araç şu anda lanelet_id={current_id} içinde")
            else:
                self.get_logger().info(" Araç hiçbir lanelet içinde değil")
            self.last_lanelet_id = current_id

        msg_out = Int32()
        msg_out.data = current_id if current_id is not None else -1
        self.pub.publish(msg_out)

class TrafficSignProcessor(Node):
    def __init__(self):
        super().__init__('traffic_sign_processor')
        
        # Trafik işareti aboneliği
        self.traffic_sign_sub = self.create_subscription(
            String, 
            '/astrid/perception/traffic_signs', 
            self.traffic_sign_callback, 
            10
        )
        self.detected_sign = None
        self.sign_detection_lanelet = None
        self.active_blocked_ids = []
        
        # Manevra tipleri
        self.manevra_linestring_types = ["park1","park2","park3","park4",
                                        "park5","park6","park7","station_manevuer"]
        
        # Manevra noktaları
        self.manevra_pointler = []
        self.manevra_yayinlandi = False
        self.collect_maneuver_points()

        # QoS Profilleri
        self.transient_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE

        )

        # Yayıncılar
        self.blocked_pub = self.create_publisher(Int32MultiArray, '/astrid/slam/blocked_lanelet_ids', self.transient_qos)
        self.manevra_pub = self.create_publisher(Float32MultiArray, '/astrid/slam/station_park_manevuer_points', self.transient_qos)

        # Lanelet aboneliği
        self.sub = self.create_subscription(Int32, '/astrid/slam/current_lanelet_id', self.lanelet_callback, 10)
        self.current_lanelet_id = None
        self.last_processed_id = -9999

    def traffic_sign_callback(self, msg):
        """Trafik işareti topic'inden gelen veriyi işler"""
        try:
            signs = json.loads(msg.data)
            if signs:
                # Yeni işaret algılandı, önceki bloklamaları temizle
                if self.active_blocked_ids:
                    self.clear_blocking()
                
                # İşaretin algılandığı anki lanelet'i kaydet
                self.sign_detection_lanelet = self.current_lanelet_id
                
                # İlk işareti al ve çevir
                raw_sign = signs[0]
                self.detected_sign = self.translate_sign(raw_sign)
                self.get_logger().info(f"Yeni trafik işareti algılandı: {raw_sign} -> {self.detected_sign}")
                
                # Bloklama işlemini başlat
                self.process_and_publish_blocking()
        except json.JSONDecodeError:
            self.get_logger().error("Geçersiz JSON formatı alındı")
        except Exception as e:
            self.get_logger().error(f"Trafik işareti işlenirken hata: {str(e)}")

    def translate_sign(self, sign):
        """İngilizce/Türkçe işaret dönüşümü"""
        sign_mapping = {
            "ileri_ve_sola_mecburi_yon": "Ileri ve sağa mecburi yon",
            "ileri_ve_saga_mecburi_yon": "Ileri ve sola mecburi yon",
            "sola_donulmez": "sağa donulmez",
            "saga_donulmez": "sola donulmez",
            "durak": "Durak",
            "park_yeri": "Park",
            "ileri": "Ileri mecburi yon",
            "girisi_olmayan_yol":"girilmez"
        }
        return sign_mapping.get(sign, sign)

    def collect_maneuver_points(self):
        """Tüm manevra noktalarını toplar"""
        for ls in map.lineStringLayer:
            if "type" in ls.attributes and ls.attributes["type"] in self.manevra_linestring_types:
                ls_type = ls.attributes["type"]
                point_list = []

                for pt in ls:
                    if "local_x" in pt.attributes and "local_y" in pt.attributes:
                        x = float(pt.attributes["local_x"])
                        y = float(pt.attributes["local_y"])
                        self.manevra_pointler.append((x, y))
                        point_list.append((x, y))

                self.get_logger().info(f"'{ls_type}' linestring'e ait {len(point_list)} nokta bulundu")

    def publish_maneuver_points(self):
        """Manevra noktalarını yayınlar"""
        if not self.manevra_pointler or self.manevra_yayinlandi:
            return

        flat_points = [coord for point in self.manevra_pointler for coord in point]
        for point in self.manevra_pointler:
            flat_points.extend(point)

        msg = Float32MultiArray()
        msg.data = flat_points
        self.manevra_pub.publish(msg)
        self.get_logger().info(f"Toplam {len(self.manevra_pointler)} manevra noktası yayınlandı")

        self.manevra_yayinlandi = True

    def process_traffic_sign(self, lanelet_id):
        """Trafik işaretine göre yasaklı lanelet'leri hesaplar"""
        if lanelet_id not in map.laneletLayer:
            self.get_logger().error(f"Lanelet ID {lanelet_id} bulunamadı!")
            return []

        current_lanelet = map.laneletLayer[lanelet_id]
        followers = routing_graph.following(current_lanelet)
        
        if not followers:
            self.get_logger().info("Takip edilebilecek lanelet yok")
            return []

        # Yön kontrolü
        sign = self.detected_sign
        if not relevant_direction_exists(sign, followers):
            self.get_logger().warning(f"Uygun yön bulunamadı: {sign}")
            return []
            
        # Özel durumlar
        if sign == "Ileriden sola mecburi yon" and straight_followed_but_turn_missing(followers, "left"):
            self.get_logger().warning("İleriden sola mecburi yön için uygun yol yok")
            return []
        if sign == "Ileriden saga mecburi yon" and straight_followed_but_turn_missing(followers, "right"):
            self.get_logger().warning("İleriden sağa mecburi yön için uygun yol yok")
            return []

        # Yasaklı laneletleri hesapla
        yasakli_ids = []
        for nxt in followers:
            td = nxt.attributes["turn_direction"] if "turn_direction" in nxt.attributes else "belirtilmemiş"
            block = False

            if sign == "saga donulmez" and td == "right":
                block = True
            elif sign == "sola donulmez" and td == "left":
                block = True
            elif sign == "girilmez" and td == "straight" :
                block = True
            elif sign == "sola mecburi yon" and td != "left":
                block = True
            elif sign == "saga mecburi yon" and td != "right":
                block = True
            elif sign == "Ileri mecburi yon" and td != "straight":
                block = True
            elif sign == "Ileri ve saga mecburi yon" and td == "left":
                block = True
            elif sign == "Ileri ve sola mecburi yon" and td == "right":
                block = True
            elif sign == "Ileriden sola mecburi yon" and td == "straight":
                second_followers = routing_graph.following(nxt)
                for sec in second_followers:
                    sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "belirtilmemiş"
                    if sec_td != "left":
                        yasakli_ids.append(sec.id)
            elif sign == "Ileriden saga mecburi yon" and td == "straight":
                second_followers = routing_graph.following(nxt)
                for sec in second_followers:
                    sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "belirtilmemiş"
                    if sec_td != "right":
                        yasakli_ids.append(sec.id)
            
            if block and sign not in ["Ileriden sola mecburi yon", "Ileriden saga mecburi yon"]:
                yasakli_ids.append(nxt.id)

        return yasakli_ids

    def process_and_publish_blocking(self):
        """Trafik işaretini işle ve bloklamayı yayınla"""
        if self.sign_detection_lanelet is None or self.detected_sign is None:
            return
            
        blocked_ids = self.process_traffic_sign(self.sign_detection_lanelet)
        self.active_blocked_ids = blocked_ids
        
        blocked_msg = Int32MultiArray()
        blocked_msg.data = blocked_ids
        self.blocked_pub.publish(blocked_msg)
        self.get_logger().info(f"Lanelet {self.sign_detection_lanelet} için yasaklı laneletler: {blocked_ids}")

    def clear_blocking(self):
        """Aktif bloklamaları temizle"""
        clear_msg = Int32MultiArray()
        clear_msg.data = []
        self.blocked_pub.publish(clear_msg)
        self.active_blocked_ids = []
        self.get_logger().info("Tüm bloklamalar kaldırıldı")

    def lanelet_callback(self, msg):
        self.current_lanelet_id = msg.data
        
        # Lanelet değiştiyse ve önceki bloklama varsa temizle
        if (self.current_lanelet_id != self.last_processed_id and 
            self.current_lanelet_id != self.sign_detection_lanelet and
            self.active_blocked_ids):
            self.clear_blocking()
            self.sign_detection_lanelet = None
            self.detected_sign = None
        
        self.last_processed_id = self.current_lanelet_id
        
        # Manevra noktalarını yayınla (sadece ilk seferde)
        if not self.manevra_yayinlandi:
            self.publish_maneuver_points()

# Ana Fonksiyon ---------------------------------------------------------------
def main():
    rclpy.init()
    
    # Düğümleri oluştur
    executor = rclpy.executors.MultiThreadedExecutor()
    matcher_node = LocalLaneletMatcher()
    processor_node = TrafficSignProcessor()
    
    executor.add_node(matcher_node)
    executor.add_node(processor_node)
    
    try:
        executor.spin()
    finally:
        matcher_node.destroy_node()
        processor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()