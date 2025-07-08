#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from shapely.geometry import Point, Polygon

import lanelet2
import lanelet2.io
import lanelet2.projection


# 🌍 Harita dosyasını yükle (projeksiyon önemli değil çünkü local_x kullanılacak)
filename = "/home/emirhan/Documents/simulation_fulltrackv6.osm"
origin = lanelet2.io.Origin(0.0, 0.0)
projector = lanelet2.projection.LocalCartesianProjector(origin)
lanelet_map = lanelet2.io.load(filename, projector)
traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)


def get_local_coords(point):
    """.osm dosyasındaki noktadan local_x, local_y koordinatlarını alır"""
    x_tag = point.attributes['local_x']
    y_tag = point.attributes['local_y']
    if x_tag is None or y_tag is None:
        return None
    return float(x_tag), float(y_tag)


class LocalLaneletMatcher(Node):
    def __init__(self):
        super().__init__('local_lanelet_matcher')

        self.subscription = self.create_subscription(
            Odometry, '/carla/hero/odometry', self.odom_callback, 10
        )

        self.pub = self.create_publisher(Int32, '/current_lanelet_id', 10)
        self.last_lanelet_id = None

    def find_current_lanelet(self, x, y):
        pt = Point(x, y)
        for ll in lanelet_map.laneletLayer:
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
        x = msg.pose.pose.position.x  # local_x
        y = msg.pose.pose.position.y  # local_y

        current_id = self.find_current_lanelet(x, y)

        if current_id != self.last_lanelet_id:
            if current_id is not None:
                self.get_logger().info(f"✅ Araç şu anda lanelet_id={current_id} içinde (x={x:.2f}, y={y:.2f})")
            else:
                self.get_logger().info(f"🚧 Araç hiçbir lanelet içinde değil (x={x:.2f}, y={y:.2f})")

            self.last_lanelet_id = current_id

        # topic'e publish et
        msg_out = Int32()
        msg_out.data = current_id if current_id is not None else -1
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalLaneletMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
