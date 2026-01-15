#!/usr/bin/env python3
"""
RViz 抓取演示启动文件
使用原有的 OpenArm URDF 模型在 RViz 中演示抓取动作
"""

import os
import xacro

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import TimerAction, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def robot_state_publisher_spawner(context: LaunchContext):
    """
    使用 Python xacro 模块处理原有的 xacro 文件
    需要手动将 xacro 模块添加到全局符号表以支持 xacro.load_yaml()
    """
    # 获取包路径
    openarm_env_description_dir = get_package_share_directory("openarm_env_description")
    
    # 构建 Xacro 文件路径
    xacro_path = os.path.join(
        openarm_env_description_dir,
        "urdf", "openarm_with_env.xacro"
    )
    
    # 关键：将 xacro 模块添加到全局符号表，这样 xacro.load_yaml() 才能工作
    xacro.global_symbols['xacro'] = xacro
    
    # 使用 Python xacro 模块处理文件
    robot_description = xacro.process_file(
        xacro_path,
        mappings={}
    ).toprettyxml(indent="  ")
    
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0,
            }],
        )
    ]


def generate_launch_description():
    # 获取包路径
    openarm_env_bringup_dir = get_package_share_directory('openarm_env_bringup')
    
    # RViz 配置文件
    rviz_config = os.path.join(openarm_env_bringup_dir, 'rviz', 'rviz_grasp_demo.rviz')

    # 使用 OpaqueFunction 来处理 xacro
    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner
    )

    return LaunchDescription([
        # 1. Robot State Publisher（使用 OpaqueFunction 处理 xacro）
        robot_state_publisher_loader,

        # 2. RViz 抓取控制器（发布 joint_states）
        Node(
            package='motion_control',
            executable='rviz_grasp_controller',
            name='rviz_grasp_controller',
            output='screen'
        ),

        # 3. RViz 可视化
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config]
                )
            ]
        ),

        # 4. 目标标记发布器
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
    ])
