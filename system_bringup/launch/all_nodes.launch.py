#!/usr/bin/env python3
"""
完整系统启动文件
启动 MuJoCo 仿真、视觉感知、运动规划和抓取控制
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 MuJoCo XML 路径
    openarm_env_desc_dir = get_package_share_directory('openarm_env_description')
    mujoco_xml_path = os.path.join(openarm_env_desc_dir, 'mujoco', 'openarm_env.xml')

    return LaunchDescription([
        # 1. MuJoCo 仿真桥接节点
        Node(
            package='openarm_env_bringup',
            executable='mujoco_ros_bridge',
            name='mujoco_ros_bridge',
            output='screen',
            parameters=[
                {'xml': mujoco_xml_path},
                # Stability / smoothing
                {'arm_command_alpha': 0.15},
                {'arm_max_step': 0.05},
                {'arm_hold_initial_pose': True},
                # 启用夹爪控制
                {'drive_gripper': True},
                # 启用可视化窗口
                {'enable_viewer': True},
            ]
        ),

        # 2. 夹爪控制器
        Node(
            package='motion_control',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen'
        ),

        # 3. 视觉感知节点（物体检测）
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='perception',
                    executable='object_detector',
                    name='vision_node',
                    output='screen',
                    parameters=[
                        {'test_mode': True}  # 使用测试模式，直接发布苹果位置
                    ]
                )
            ]
        ),

        # 4. 抓取状态机（核心控制）
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

        # 5. 自动抓取协调器
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='motion_control',
                    executable='auto_grasp_coordinator',
                    name='auto_grasp_coordinator',
                    output='screen'
                )
            ]
        ),
    ])
