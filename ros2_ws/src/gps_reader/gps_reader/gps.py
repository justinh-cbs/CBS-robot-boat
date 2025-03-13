"""
ROS2 node for reading and publishing GPS data
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header
import gpsd
from gpsd import NoFixError


class GpsNode(Node):
    def __init__(self):
        super().__init__("gps_node")
        self.publisher_ = self.create_publisher(NavSatFix, "gps/fix", 10)
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Connecting to GPSD...")
        try:
            gpsd.connect()
            self.get_logger().info("Connected to GPSD successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPSD: {str(e)}")
            raise

        self.last_fix = False
        self.consecutive_errors = 0
        self.MAX_ERRORS = 10

    def timer_callback(self):
        try:
            gps_packet = gpsd.get_current()

            # Check if we have a fix
            if gps_packet.mode >= 2:
                msg = NavSatFix()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "gps"

                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS

                msg.latitude = gps_packet.lat
                msg.longitude = gps_packet.lon

                # If we have altitude (3D fix)
                if gps_packet.mode >= 3:
                    msg.altitude = gps_packet.alt

                try:
                    # PDOP (Position Dilution of Precision)
                    pdop = gps_packet.pdop
                    covariance = pdop * pdop if pdop else 100.0
                    msg.position_covariance = [
                        covariance,
                        0.0,
                        0.0,
                        0.0,
                        covariance,
                        0.0,
                        0.0,
                        0.0,
                        covariance,
                    ]
                    msg.position_covariance_type = (
                        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                    )
                except AttributeError:
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.publisher_.publish(msg)

                if not self.last_fix:
                    self.get_logger().info(f"Got GPS fix! Mode: {gps_packet.mode}D")
                    self.last_fix = True
                self.consecutive_errors = 0

            else:
                if self.last_fix:
                    self.get_logger().warning("Lost GPS fix")
                    self.last_fix = False
                self.get_logger().debug(
                    f"Waiting for GPS fix, current mode: {gps_packet.mode}"
                )

        except NoFixError:
            if self.last_fix:
                self.get_logger().warning("Lost GPS fix")
                self.last_fix = False
            self.get_logger().debug("Waiting for GPS fix")

        except Exception as e:
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.MAX_ERRORS:
                self.get_logger().warning(
                    f"GPS read failed {self.consecutive_errors} times: {str(e)}"
                )
            else:
                self.get_logger().debug(f"GPS read error: {str(e)}")

            # try to reconnect if we've had too many errors
            if self.consecutive_errors >= self.MAX_ERRORS * 2:
                self.get_logger().info("Attempting to reconnect to GPSD...")
                try:
                    gpsd.connect()
                    self.consecutive_errors = 0
                    self.get_logger().info("Reconnected to GPSD successfully")
                except Exception as e:
                    self.get_logger().error(f"Failed to reconnect to GPSD: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    gps_node = GpsNode()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
