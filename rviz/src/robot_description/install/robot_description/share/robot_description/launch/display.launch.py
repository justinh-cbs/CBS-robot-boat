from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    visualizer = Node(
        package='robot_description',
        executable='robot_visualizer',
        name='robot_visualizer',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                            description='Absolute path to rviz config file'),
        robot_state_publisher,
        rviz,
        visualizer
    ])
