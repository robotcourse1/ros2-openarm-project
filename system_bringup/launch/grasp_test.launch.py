#!/usr/bin/env python3
"""
简化启动文件 - 用于测试抓取功能
只启动 MuJoCo 仿真和抓取控制（不包含视觉）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 MuJoCo XML 路径
    try:
        openarm_env_desc_dir = get_package_share_directory('openarm_env_description')
        mujoco_xml_path = os.path.join(openarm_env_desc_dir, 'mujoco', 'openarm_env.xml')
    except:
        mujoco_xml_path = ''

    return LaunchDescription([
        # 1. MuJoCo 仿真桥接节点
        Node(
            package='openarm_env_bringup',
            executable='mujoco_ros_bridge',
            name='mujoco_ros_bridge',
            output='screen',
            parameters=[{'xml': mujoco_xml_path}] if mujoco_xml_path else []
        ),

        # 2. 夹爪控制器
        Node(
            package='motion_control',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen'
        ),

        # 3. 抓取状态机（核心控制）
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='motion_control',
                    executable='grasp_state_machine',
                    name='grasp_state_machine',
                    output='screen'
                )
            ]
        ),
    ])
