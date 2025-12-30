#!/usr/bin/env python3
"""
双臂协调抓取启动文件
集成 openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    use_bimanual_arg = DeclareLaunchArgument(
        'use_bimanual',
        default_value='true',
        description='是否使用双臂模式'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base',
        description='机器人基座链接名称'
    )
    
    planner_id_arg = DeclareLaunchArgument(
        'planner_id',
        default_value='RRTConnectkConfigDefault',
        description='OMPL规划器ID'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.3',
        description='最大速度缩放因子'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='0.3',
        description='最大加速度缩放因子'
    )
    
    coordination_mode_arg = DeclareLaunchArgument(
        'coordination_mode',
        default_value='sequential',
        description='协调模式: sequential(顺序) 或 parallel(并行)'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='命名空间（用于 openarm.bimanual.launch.py 的命名空间配置）'
    )
    
    fallback_to_single_arm_arg = DeclareLaunchArgument(
        'fallback_to_single_arm',
        default_value='true',
        description='不稳定时先单臂'
    )
    
    # 包含 openarm.bimanual.launch.py（原始项目的双臂launch配置）
    openarm_bimanual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openarm_bringup'),
                'launch/openarm.bimanual.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_moveit': 'true',
        }.items()
    )
    
    # 双臂抓取规划节点
    bimanual_grasp_node = Node(
        package='motion_control',
        executable='bimanual_grasp_planner',
        name='bimanual_grasp_planner',
        output='screen',
        parameters=[{
            'use_bimanual': LaunchConfiguration('use_bimanual'),
            'base_link': LaunchConfiguration('base_link'),
            'planner_id': LaunchConfiguration('planner_id'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'coordination_mode': LaunchConfiguration('coordination_mode'),
            'namespace': LaunchConfiguration('namespace'),
            'fallback_to_single_arm': LaunchConfiguration('fallback_to_single_arm'),
            'pre_grasp_offset_z': 0.15,
            'grasp_offset_z': 0.02,
            'lift_height': 0.2,
            'place_position': [0.5, 0.3, 0.3],
        }],
    )
    
    # 声明 use_sim_time 和 use_fake_hardware 参数（用于 openarm.bimanual.launch.py）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='是否使用虚拟硬件'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_fake_hardware_arg,
        use_bimanual_arg,
        base_link_arg,
        planner_id_arg,
        max_velocity_arg,
        max_acceleration_arg,
        coordination_mode_arg,
        namespace_arg,
        fallback_to_single_arm_arg,
        LogInfo(msg=['启动双臂抓取规划节点（集成 openarm.bimanual.launch.py）']),
        openarm_bimanual_launch,  # 先启动原始项目的双臂launch配置
        bimanual_grasp_node,
    ])

