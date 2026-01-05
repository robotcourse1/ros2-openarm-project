from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calibration',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
        )
    ])
