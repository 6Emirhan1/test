import lanelet2
import lanelet2.io
import lanelet2.projection
import lanelet2.traffic_rules
import lanelet2.routing

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

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

# Kullanıcıdan giriş
detected_sign = input("Tespit edilen levhayı girin: ").strip()
lanelet_id = int(input("Mevcut lanelet ID'sini girin: ").strip())

if lanelet_id not in map.laneletLayer:
    raise ValueError(f"Lanelet ID {lanelet_id} haritada bulunamadı.")

current_lanelet = map.laneletLayer[lanelet_id]
followers = routing_graph.following(current_lanelet)

if not followers:
    print(f"Lanelet {lanelet_id} için takip edilebilecek lanelet yok.")
    exit(0)

allowed, blocked, yasakli_lanelet_idler = [], [], []

# ✅ Yön uygun mu kontrolü
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

# Mecburi yön sonrası gerekli yön yok mu?
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

# Uygun yön yoksa levhayı dikkate alma
if not relevant_direction_exists(detected_sign, followers):
    print(f"Uyarı: '{detected_sign}' levhası için uygun yön bulunamadı. Levha dikkate alınmadı.")
    detected_sign = None
elif detected_sign == "Ileriden sola mecburi yon" and straight_followed_but_turn_missing(followers, "left"):
    print("Uyarı: 'İleriden sola mecburi yön' için ileri var ama sola giden yol yok. Levha dikkate alınmadı.")
    detected_sign = None
elif detected_sign == "Ileriden saga mecburi yon" and straight_followed_but_turn_missing(followers, "right"):
    print("Uyarı: 'İleriden sağa mecburi yön' için ileri var ama sağa giden yol yok. Levha dikkate alınmadı.")
    detected_sign = None

manevra_pointler=[]

# Ana işlem döngüsü
for nxt in followers:
    td = nxt.attributes["turn_direction"] if "turn_direction" in nxt.attributes else "belirtilmemiş"
    block = False

    if detected_sign == "saga donulmez":
        if td == "right": block = True
    elif detected_sign == "sola donulmez":
        if td == "left": block = True
    elif detected_sign == "sola mecburi yon":
        if td != "left": block = True
    elif detected_sign == "saga mecburi yon":
        if td != "right": block = True
    elif detected_sign == "Ileri mecburi yon":
        if td != "straight": block = True
    elif detected_sign == "Ileri ve saga mecburi yon":
        if td == "left": block = True
    elif detected_sign == "Ileri ve sola mecburi yon":
        if td == "right": block = True
    elif detected_sign == "Ileriden sola mecburi yon":
        if td == "straight":
            second_followers = routing_graph.following(nxt)
            for sec in second_followers:
                sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "belirtilmemiş"
                if sec_td != "left":
                    blocked.append((sec.id, sec_td))
                    yasakli_lanelet_idler.append(sec.id)
                else:
                    allowed.append((sec.id, sec_td))
    elif detected_sign == "Ileriden saga mecburi yon":
        if td == "straight":
            second_followers = routing_graph.following(nxt)
            for sec in second_followers:
                sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "belirtilmemiş"
                if sec_td != "right":
                    blocked.append((sec.id, sec_td))
                    yasakli_lanelet_idler.append(sec.id)
                else:
                    allowed.append((sec.id, sec_td))

    if detected_sign not in ["Ileriden sola mecburi yon", "Ileriden saga mecburi yon"]:
        if block:
            blocked.append((nxt.id, td))
            yasakli_lanelet_idler.append(nxt.id)
        else:
            allowed.append((nxt.id, td))


# Durak manevrası ve park manevrası için Döngü

    bulundu_mu = False
    linestring_type = input("\nAranacak linestring 'type' değerini girin (örnek: durak1, durak2, manevra): ").strip()

    if detected_sign == "Durak":
        print(f"\n'Durak' levhası tespit edildi. '{linestring_type}' isimli linestring aranıyor...")

        for ls in map.lineStringLayer:
            if "type" in ls.attributes and ls.attributes["type"] == linestring_type:
                print(f"→ '{linestring_type}' bulundu! Noktalar:")

                for pt in ls:
                    if "local_x" in pt.attributes and "local_y" in pt.attributes:
                        x = float(pt.attributes["local_x"])
                        y = float(pt.attributes["local_y"])
                        point = (x, y)
                        manevra_pointler.append(point)
                        print(f"   • Point: x={x}, y={y}")
                    else:
                        print("   • Uyarı: local_x veya local_y etiketi bulunamadı. Nokta atlandı.")

                bulundu_mu = True
                break  # sadece ilk eşleşen linestring alınır

        if not bulundu_mu:
            print(f"Uyarı: type='{linestring_type}' olan bir linestring bulunamadı.")

        print(f"\nToplam manevra noktası sayısı: {len(manevra_pointler)}")

    elif detected_sign == "Park yeri":
        print(f"\n'Park yeri' levhası tespit edildi. '{linestring_type}' adlı linestring aranıyor...")

        for ls in map.lineStringLayer:
            if "type" in ls.attributes and ls.attributes["type"] == linestring_type:
                print(f"→ '{linestring_type}' bulundu! Noktalar:")

                for pt in ls:
                    if "local_x" in pt.attributes and "local_y" in pt.attributes:
                        x = float(pt.attributes["local_x"])
                        y = float(pt.attributes["local_y"])
                        point = (x, y)
                        manevra_pointler.append(point)
                        print(f"   • Point: x={x}, y={y}")
                    else:
                        print("   • Uyarı: local_x veya local_y etiketi bulunamadı. Nokta atlandı.")

                bulundu_mu = True
                break

        if not bulundu_mu:
            print(f"Uyarı: type='{linestring_type}' olan bir linestring bulunamadı.")

        print(f"\nToplam manevra noktası sayısı: {len(manevra_pointler)}")


print(f"manevra için pointler:{manevra_pointler}")
# Sonuç yazdır
print(f"\nLevha: {detected_sign if detected_sign else 'YOK (yön bulunamadı)'}")
print(f"Mevcut lanelet ID: {lanelet_id}")

print("\n→ İzin verilen follower lanelet'ler:")
for ll_id, td in allowed:
    print(f"   • ID: {ll_id}, turn_direction: {td}")

print("\n→ Bloklanan follower lanelet'ler:")
for ll_id, td in blocked:
    print(f"   • ID: {ll_id}, turn_direction: {td}")

print("\n→ Yasaklı lanelet ID listesi:")
print(yasakli_lanelet_idler)

# === ROS 2 Publisher ===
class LaneletPublisher(Node):
    def __init__(self, yasakli_ids):
        super().__init__('lanelet_block_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.publisher_ = self.create_publisher(Int32MultiArray, 'blocked_lanelet_ids', qos_profile)

        msg = Int32MultiArray()
        msg.data = yasakli_ids

        self.publisher_.publish(msg)
        self.get_logger().info(f"Yasaklı lanelet ID'leri yayınlandı: {yasakli_ids}")

# === Manevra Noktaları Publisher Node ===
class ManevraPointPublisher(Node):
    def __init__(self, point_list):
        super().__init__('manevra_point_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.publisher_ = self.create_publisher(Float32MultiArray, 'detected_pose', qos_profile)

        # Listeyi düzleştir (x1, y1, x2, y2, ...)
        flattened = [coord for point in point_list for coord in point]

        msg = Float32MultiArray()
        msg.data = flattened

        self.publisher_.publish(msg)
        for i in range(len(point_list)):
            x, y = point_list[i]
            self.get_logger().info(f"Point {i+1}: x={x}, y={y}")

# === Main ===
def main():
    rclpy.init()

    # Yasaklı lanelet ID'lerini yayınlayan node
    lanelet_node = LaneletPublisher(yasakli_lanelet_idler)

    # Manevra noktalarını yayınlayan node
    point_node = ManevraPointPublisher(manevra_pointler)

    # Her iki node birlikte çalışsın
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lanelet_node)
    executor.add_node(point_node)

    try:
        executor.spin()
    finally:
        lanelet_node.destroy_node()
        point_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
