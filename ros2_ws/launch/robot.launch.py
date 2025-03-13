from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Joystick teleop node (converts joy messages to cmd_vel)
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="joy_teleop_node",
                parameters=[
                    {
                        # Axes mapping
                        "axis_linear.x": 1,  # Left stick up/down
                        "axis_angular.yaw": 3,  # Right stick left/right
                        # Speed scales
                        "scale_linear.x": 0.5,
                        "scale_angular.yaw": 0.5,
                        # Button configuration
                        "require_enable_button": False,
                        "enable_turbo_button": 4,  # RB button
                        "scale_linear_turbo.x": 1.0,
                        "scale_angular_turbo.yaw": 1.0,
                    }
                ],
            ),
            # GPS node
            Node(
                package="gps_reader",
                executable="gps",
                name="gps_node",
            ),
            # IMU
            Node(
                package="imu",
                executable="imu_node",
                name="imu_node",
                parameters=[
                    {
                        "frame_id": "base_imu_link",
                    }
                ],
            ),
            # IMU Fusion node
            Node(
                package="imu",
                executable="imu_fusion",
                name="imu_fusion_node",
                output="screen",
            ),
            # Adjust PID here rather than in the node
            Node(
                package="waypoint_nav",
                executable="waypoint_navigator",
                name="waypoint_navigator",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "switch_button": 5,  # RB button for switching modes
                        "waypoint_radius": 2.0,  # meters
                        "max_linear_speed": 0.5,  # m/s
                        "max_angular_speed": 0.5,  # rad/s
                        "pid_linear_kp": 0.5,
                        "pid_linear_ki": 0.0,
                        "pid_linear_kd": 0.1,
                        "pid_angular_kp": 1.0,
                        "pid_angular_ki": 0.0,
                        "pid_angular_kd": 0.1,
                    }
                ],
            ),
            # Motor control node
            Node(
                package="jetson_motor_control",
                executable="motor_control",
                name="motor_control_node",
            ),
        ]
    )
