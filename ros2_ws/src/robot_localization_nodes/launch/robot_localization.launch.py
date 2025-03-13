from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ekf_config_path = os.path.join(
        get_package_share_directory("robot_localization_nodes"),
        "config",
        "ekf_config.yaml",
    )

    navsat_params = {
        "frequency": 30.0,
        "magnetic_declination_radians": 0.0,
        "yaw_offset": 0.0,
        "zero_altitude": True,
        "broadcast_utm_transform": True,
        "broadcast_utm_transform_as_parent_frame": True,
        "publish_filtered_gps": True,
        "use_odometry_yaw": False,
        "wait_for_datum": False,
        "frame_id": "gps",
    }

    return LaunchDescription(
        [
            Node(
                package="robot_localization_nodes",
                executable="robot_odometry",
                name="robot_odometry",
                parameters=[{"use_gps": True, "use_imu": True}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config_path],
                remappings=[("/odometry/filtered", "/odom")],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[navsat_params],
                remappings=[
                    ("imu", "imu"),
                    ("gps/fix", "gps/fix"),
                    ("odometry/filtered", "odom"),
                    ("odometry/gps", "odometry/gps"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_imu_link"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_gps_publisher",
                arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "gps"],
            ),
        ]
    )
