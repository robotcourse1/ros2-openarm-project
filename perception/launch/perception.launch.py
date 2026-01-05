from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='object_detector', # 这里要和 setup.py 里的 console_scripts 名字一致
            name='vision_node',
            output='screen',
            emulate_tty=True
        ),
    ])
