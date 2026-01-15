#!/usr/bin/env python3
"""
完整系统启动文件（带 RViz 可视化）
启动 MuJoCo 仿真、RViz、视觉感知、运动规划和抓取控制

注意：由于 xacro 文件兼容性问题，RViz 不显示完整机器人模型，
但会显示末端执行器位置、目标标记和抓取轨迹。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    openarm_env_desc_dir = get_package_share_directory('openarm_env_description')
    openarm_env_bringup_dir = get_package_share_directory('openarm_env_bringup')
    
    # MuJoCo XML 路径
    mujoco_xml_path = os.path.join(openarm_env_desc_dir, 'mujoco', 'openarm_env.xml')
    
    # RViz 配置文件
    rviz_config_path = os.path.join(openarm_env_bringup_dir, 'rviz', 'openarm_env.rviz')

    return LaunchDescription([
        # 1. MuJoCo 仿真桥接节点（平滑运动参数）
        Node(
            package='openarm_env_bringup',
            executable='mujoco_ros_bridge',
            name='mujoco_ros_bridge',
            output='screen',
            parameters=[
                {'xml': mujoco_xml_path},
                # 平滑运动参数 - 降低速度以便观察
                {'arm_command_alpha': 0.08},  # 更慢的响应
                {'arm_max_step': 0.02},       # 更小的步长
                {'arm_hold_initial_pose': True},
                {'drive_gripper': True},
                {'enable_viewer': True},
            ]
        ),

        # 2. RViz 可视化（简化模式，不需要 robot_description）
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path]
                )
            ]
        ),

        # 3. 目标标记发布器（在 RViz 中显示目标位置）
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='motion_control',
                    executable='target_marker_publisher',
                    name='target_marker_publisher',
                    output='screen'
                )
            ]
        ),

        # 4. 夹爪控制器
        Node(
            package='motion_control',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen'
        ),

        # 5. 视觉感知节点（物体检测）
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='perception',
                    executable='object_detector',
                    name='vision_node',
                    output='screen',
                    parameters=[
                        {'test_mode': True}
                    ]
                )
            ]
        ),

        # 6. 抓取状态机（核心控制）
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

        # 7. 自动抓取协调器
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
