#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from waypoint_nav_interfaces.srv import AddWaypoint, ClearWaypoints, GetWaypoints
from sensor_msgs.msg import NavSatFix
from foxglove_msgs.msg import GeoJSON
import csv
import json
import re
import time


class WaypointLoader(Node):
    def __init__(self):
        super().__init__("waypoint_loader")

        self.declare_parameter("csv_path", "")
        self.declare_parameter("path_color", "#0000FF")
        self.declare_parameter("path_weight", 4)
        self.declare_parameter("path_opacity", 1.0)

        transient_local_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1000,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        self.waypoint_gps_pub = self.create_publisher(
            NavSatFix, "waypoint_positions", qos_profile=transient_local_qos
        )
        self.geojson_pub = self.create_publisher(
            GeoJSON, "path_geojson", qos_profile=transient_local_qos
        )

        self.add_waypoint_client = self.create_client(AddWaypoint, "add_waypoint")
        self.clear_waypoints_client = self.create_client(
            ClearWaypoints, "clear_waypoints"
        )

        self.get_waypoints_srv = self.create_service(
            GetWaypoints, "get_waypoints", self.get_waypoints_callback
        )

        self.waypoints = []
        self.geojson_cache = None

        ready = True
        if not self.add_waypoint_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("add_waypoint service not available")
            ready = False
        if not self.clear_waypoints_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("clear_waypoints service not available")
            ready = False

        if ready:
            self.load_waypoints()
            self.publish_visualization()

    def get_waypoints_callback(self, request, response):
        """Service callback to get waypoints on demand"""
        self.publish_visualization()
        response.success = True
        response.message = f"Published {len(self.waypoints)} waypoints"
        return response

    def create_path_geojson(self):
        """Create GeoJSON feature collection for waypoints"""
        features = []

        if len(self.waypoints) >= 2:
            for i in range(len(self.waypoints) - 1):
                features.append(
                    {
                        "type": "Feature",
                        "properties": {
                            "name": f"Waypoint Segment {i+1}",
                            "style": {
                                "color": self.get_parameter("path_color").value,
                                "weight": self.get_parameter("path_weight").value,
                                "opacity": self.get_parameter("path_opacity").value,
                            },
                        },
                        "geometry": {
                            "type": "LineString",
                            "coordinates": [self.waypoints[i], self.waypoints[i + 1]],
                        },
                    }
                )

        if not features:
            return None

        return {"type": "FeatureCollection", "features": features}

    def publish_visualization(self):
        """Publish visualization data once"""
        if not self.waypoints:
            return

        clear_msg = NavSatFix()
        clear_msg.header.frame_id = "map"
        clear_msg.header.stamp = self.get_clock().now().to_msg()
        clear_msg.status.status = -1
        self.waypoint_gps_pub.publish(clear_msg)

        for i, (lon, lat) in enumerate(self.waypoints):
            gps_msg = NavSatFix()
            gps_msg.header.frame_id = "map"
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = 0.0
            gps_msg.status.status = i
            self.waypoint_gps_pub.publish(gps_msg)
            time.sleep(0.1)

        if self.geojson_cache is None:
            geojson = self.create_path_geojson()
            if geojson:
                self.geojson_cache = json.dumps(geojson)

        if self.geojson_cache:
            msg = GeoJSON()
            msg.geojson = self.geojson_cache
            self.geojson_pub.publish(msg)

    def parse_wkt_linestring(self, wkt_string):
        """Parse WKT LINESTRING format and return list of [lon, lat] coordinates"""
        coords_match = re.search(r"LINESTRING \((.*)\)", wkt_string)
        if not coords_match:
            raise ValueError(f"Invalid WKT LINESTRING format: {wkt_string}")

        coords_str = coords_match.group(1)
        coords_pairs = coords_str.split(",")

        waypoints = []
        for pair in coords_pairs:
            lon, lat = map(float, pair.strip().split())
            waypoints.append([lon, lat])

        return waypoints

    def load_waypoints(self):
        """Load waypoints from CSV file and send them to navigation system"""
        csv_path = self.get_parameter("csv_path").value
        if not csv_path:
            self.get_logger().error("No CSV path specified")
            return

        try:
            self.get_logger().info("Clearing existing waypoints...")
            req = ClearWaypoints.Request()
            future = self.clear_waypoints_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            self.get_logger().info(f"Loading waypoints from {csv_path}")
            with open(csv_path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if "WKT" not in row:
                        self.get_logger().error("CSV file must have a WKT column")
                        return

                    try:
                        self.waypoints = self.parse_wkt_linestring(row["WKT"])
                        self.get_logger().info(
                            f"Loaded {len(self.waypoints)} waypoints"
                        )

                    except ValueError as e:
                        self.get_logger().error(f"Error parsing WKT: {str(e)}")
                        continue

                    for i, waypoint in enumerate(self.waypoints):
                        lon, lat = waypoint
                        self.get_logger().info(
                            f"Adding waypoint {i+1}/{len(self.waypoints)}: lon={lon}, lat={lat}"
                        )
                        req = AddWaypoint.Request()
                        req.latitude = lat
                        req.longitude = lon

                        future = self.add_waypoint_client.call_async(req)
                        rclpy.spin_until_future_complete(self, future)

            self.get_logger().info("Finished loading waypoints")

        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found: {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = WaypointLoader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
