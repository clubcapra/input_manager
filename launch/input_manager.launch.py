from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('input_manager')
    default_config = os.path.join(pkg_share, 'config', 'default_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to the input manager YAML config file'
        ),
        DeclareLaunchArgument(
            'no_gui',
            default_value='false',
            description='Run without the Tkinter GUI (true/false)'
        ),
        Node(
            package='input_manager',
            executable='input_manager',
            name='input_manager',
            output='screen',
            parameters=[{
                'config': LaunchConfiguration('config'),
                'no_gui': LaunchConfiguration('no_gui'),
            }],
        ),
    ])