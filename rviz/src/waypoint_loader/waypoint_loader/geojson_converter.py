#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from foxglove_msgs.msg import GeoJSON
import json
from collections import deque
from datetime import datetime


class GeoJSONConverter(Node):
    def __init__(self):
        super().__init__("geojson_converter")

        # Parameters
        self.declare_parameter("path_memory_size", 100)
        self.declare_parameter("waypoint_color", "#0000FF")  # Red for waypoints
        # self.declare_parameter('path_color', '#0000FF')     # Blue for current path
        self.declare_parameter("waypoint_weight", 4)
        # self.declare_parameter('path_weight', 2)
        self.declare_parameter("waypoint_opacity", 1.0)
        # self.declare_parameter('path_opacity', 0.8)

        # Store waypoints and current path points separately
        self.waypoints = []
        # self.current_path = deque(maxlen=self.get_parameter('path_memory_size').value)

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            NavSatFix, "waypoint_positions", self.waypoint_callback, 10
        )

        # self.path_sub = self.create_subscription(
        #     NavSatFix,
        #     'waypoint_path',
        #     self.path_callback,
        #     10
        # )

        # Publisher for GeoJSON
        self.geojson_pub = self.create_publisher(GeoJSON, "path_geojson", 10)

        # Timer for publishing GeoJSON
        self.create_timer(1.0, self.publish_geojson)

        self.get_logger().info("GeoJSON converter node initialized")

    def waypoint_callback(self, msg: NavSatFix):
        """Handle incoming waypoints"""
        point = (msg.longitude, msg.latitude)
        if point not in self.waypoints:
            self.waypoints.append(point)
            self.get_logger().debug(f"Added waypoint: {point}")

    # def path_callback(self, msg: NavSatFix):
    #     """Handle incoming path points"""
    #     self.current_path.append((msg.longitude, msg.latitude))

    def create_path_geojson(self):
        """Create GeoJSON feature collection for both waypoints and current path"""
        features = []

        # Create features for waypoint connections
        if len(self.waypoints) >= 2:
            for i in range(len(self.waypoints) - 1):
                features.append(
                    {
                        "type": "Feature",
                        "properties": {
                            "name": f"Waypoint Segment {i+1}",
                            "style": {
                                "color": self.get_parameter("waypoint_color").value,
                                "weight": self.get_parameter("waypoint_weight").value,
                                "opacity": self.get_parameter("waypoint_opacity").value,
                            },
                        },
                        "geometry": {
                            "type": "LineString",
                            "coordinates": [self.waypoints[i], self.waypoints[i + 1]],
                        },
                    }
                )

        # Create features for current path segment
        # if len(self.current_path) >= 2:
        #     path_points = list(self.current_path)
        #     for i in range(len(path_points) - 1):
        #         features.append({
        #             "type": "Feature",
        #             "properties": {
        #                 "name": f"Current Path Segment {i+1}",
        #                 "style": {
        #                     "color": self.get_parameter('path_color').value,
        #                     "weight": self.get_parameter('path_weight').value,
        #                     "opacity": self.get_parameter('path_opacity').value
        #                 }
        #             },
        #             "geometry": {
        #                 "type": "LineString",
        #                 "coordinates": [path_points[i], path_points[i+1]]
        #             }
        #         })

        if not features:
            return None

        return {"type": "FeatureCollection", "features": features}

    def publish_geojson(self):
        """Publish path as GeoJSON"""
        geojson = self.create_path_geojson()
        if geojson:
            msg = GeoJSON()
            msg.geojson = json.dumps(geojson)
            self.geojson_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GeoJSONConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
