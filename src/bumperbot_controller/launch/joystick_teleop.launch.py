from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('bumperbot_controller'),
        'config',
        'joy_config.yaml'
    )
    config_dir_2 = os.path.join(
        get_package_share_directory('bumperbot_controller'),
        'config',
        'joy_teleop.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[config_dir],
        output='screen'
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[config_dir_2],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        joy_teleop_node
    ])
