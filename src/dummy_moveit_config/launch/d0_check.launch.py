"""Run MoveIt model/FK/self-collision diagnostics without planning or execution."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc = Path(get_package_share_directory('dummy-ros2_description'))
    config = Path(get_package_share_directory('dummy_moveit_config'))
    return LaunchDescription([
        Node(package='dummy_moveit_config', executable='d0_model_check',
             output='screen', parameters=[{
                 'robot_description': xacro.process_file(
                     str(desc / 'urdf/dummy-ros2.xacro')).toxml(),
                 'robot_description_semantic':
                     (config / 'config/dummy-ros2.srdf').read_text(),
             }])
    ])
