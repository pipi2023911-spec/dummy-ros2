"""D0 display only: no move_group, controller or hardware commands."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description = Path(get_package_share_directory('dummy-ros2_description'))
    config = Path(get_package_share_directory('dummy_moveit_config'))
    urdf = xacro.process_file(str(description / 'urdf/dummy-ros2.xacro')).toxml()
    srdf = (config / 'config/dummy-ros2.srdf').read_text()
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(config / 'config/d0_model.rviz')],
             parameters=[{'robot_description': urdf,
                          'robot_description_semantic': srdf}]),
    ])
