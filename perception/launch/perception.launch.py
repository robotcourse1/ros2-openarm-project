from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='vision_node',
            name='vision_node',
            output='screen'
        )
    ])
