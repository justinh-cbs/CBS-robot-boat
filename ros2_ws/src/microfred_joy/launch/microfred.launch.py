from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {
                        "dev": "/dev/input/js0",
                        "deadzone": 0.1,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="joy_teleop_node",
                parameters=[
                    {
                        # Correct parameter names for axes
                        "axis_linear.x": 1,  # Left stick up/down
                        "axis_angular.yaw": 3,  # Right stick left/right
                        # Scale parameters
                        "scale_linear.x": 0.5,
                        "scale_angular.yaw": 0.5,
                        # Disable enable button requirement
                        "require_enable_button": False,
                        # Optional turbo settings
                        "enable_turbo_button": 4,  # RB button
                        "scale_linear_turbo.x": 1.0,
                        "scale_angular_turbo.yaw": 1.0,
                    }
                ],
            ),
        ]
    )
