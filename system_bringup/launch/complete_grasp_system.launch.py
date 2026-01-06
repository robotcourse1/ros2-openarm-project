#!/usr/bin/env python3
"""
完整抓取系统启动文件
整合MuJoCo仿真、视觉检测、抓取规划和系统监控
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明launch参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz可视化'
    )
    
    use_real_vision_arg = DeclareLaunchArgument(
        'use_real_vision',
        default_value='true',
        description='使用真实视觉检测(true)或模拟检测(false)'
    )
    
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='left',
        description='抓取臂类型: left, right, 或 bimanual'
    )
    
    # 获取配置
    use_rviz = LaunchConfiguration('use_rviz')
    use_real_vision = LaunchConfiguration('use_real_vision')
    arm_type = LaunchConfiguration('arm_type')
    
    # 1. MuJoCo环境 + ROS2桥接
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openarm_env_bringup'),
                'launch',
                'mujoco_env_with_bridge.launch.py'
            ])
        ]),
        launch_arguments={'static': 'true'}.items()
    )
    
    # 2. RViz可视化 (延迟3秒启动)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('openarm_env_bringup'),
        'rviz',
        'openarm_env.rviz'
    ])
    
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                condition=IfCondition(use_rviz),
                output='screen'
            )
        ]
    )
    
    # 3. 视觉检测节点
    real_vision_node = Node(
        package='perception',
        executable='real_object_detector',
        name='vision_detector',
        output='screen',
        condition=IfCondition(use_real_vision)
    )
    
    simulated_vision_node = Node(
        package='perception',
        executable='object_detector',
        name='vision_detector',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_real_vision, "' == 'false'"]))
    )
    
    # 4. 抓取规划节点
    left_grasp_planner = Node(
        package='motion_control',
        executable='grasp_planner',
        name='left_grasp_planner',
        output='screen',
        condition=IfCondition(PythonExpression(["'", arm_type, "' == 'left'"]))
    )
    
    right_grasp_planner = Node(
        package='motion_control',
        executable='grasp_planner',
        name='right_grasp_planner',
        parameters=[{'arm_name': 'right_arm'}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", arm_type, "' == 'right'"]))
    )
    
    bimanual_grasp_planner = Node(
        package='motion_control',
        executable='bimanual_grasp_planner',
        name='bimanual_grasp_planner',
        output='screen',
        condition=IfCondition(PythonExpression(["'", arm_type, "' == 'bimanual'"]))
    )
    
    # 5. 系统监控节点
    system_monitor = Node(
        package='system_bringup',
        executable='system_monitor',
        name='system_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        use_real_vision_arg,
        arm_type_arg,
        mujoco_launch,
        rviz_node,
        real_vision_node,
        simulated_vision_node,
        left_grasp_planner,
        right_grasp_planner,
        bimanual_grasp_planner,
        system_monitor,
    ])
