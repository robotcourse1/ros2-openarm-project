from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="perception", executable="minimal", name="perception_node", output="screen"),
        Node(package="calibration", executable="minimal", name="calibration_node", output="screen"),
        Node(package="motion_control", executable="minimal", name="motion_control_node", output="screen")
    ])
