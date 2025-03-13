from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")

    default_model_path = os.path.join(pkg_share, "urdf", "robot.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "urdf.rviz")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )

    static_transform_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_gps",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "gps"],
        ),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", LaunchConfiguration("model")]),
                "use_sim_time": False,
            }
        ],
    )

    ros_bridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[
            {
                "port": 9090,
                "address": "0.0.0.0",
                "delay_between_messages": 0,
                "max_message_size": 10000000,
            }
        ],
        output="screen",
    )

    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        parameters=[{"topics_glob": "", "services_glob": "", "params_glob": ""}],
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher,
            *static_transform_nodes,
            ros_bridge,
            rosapi,
        ]
    )
