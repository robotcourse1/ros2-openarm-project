#!/usr/bin/env python3
"""
启动抓取规划与控制节点的launch文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述"""
    
    # 声明启动参数
    arm_group_arg = DeclareLaunchArgument(
        'arm_group',
        default_value='left_arm',
        description='规划组名称 (left_arm 或 right_arm)'
    )
    
    gripper_group_arg = DeclareLaunchArgument(
        'gripper_group',
        default_value='left_hand',
        description='夹爪组名称 (left_hand 或 right_hand)'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base',
        description='机器人基座链接名称'
    )
    
    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='openarm_left_link7',
        description='末端执行器链接名称'
    )
    
    planner_id_arg = DeclareLaunchArgument(
        'planner_id',
        default_value='RRTConnectkConfigDefault',
        description='OMPL规划器ID'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.3',
        description='最大速度缩放因子 (0.0-1.0)'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='0.3',
        description='最大加速度缩放因子 (0.0-1.0)'
    )
    
    pre_grasp_offset_z_arg = DeclareLaunchArgument(
        'pre_grasp_offset_z',
        default_value='0.15',
        description='预抓取位置在物体上方的高度 (米)'
    )
    
    grasp_offset_z_arg = DeclareLaunchArgument(
        'grasp_offset_z',
        default_value='0.02',
        description='抓取位置相对物体的高度偏移 (米)'
    )
    
    lift_height_arg = DeclareLaunchArgument(
        'lift_height',
        default_value='0.2',
        description='提升高度 (米)'
    )
    
    place_position_arg = DeclareLaunchArgument(
        'place_position',
        default_value='[0.5, 0.3, 0.3]',
        description='放置位置 [x, y, z] (米)'
    )
    
    # 抓取规划节点
    grasp_planner_node = Node(
        package='motion_control',
        executable='grasp_planner',
        name='grasp_planner',
        output='screen',
        parameters=[{
            'arm_group': LaunchConfiguration('arm_group'),
            'gripper_group': LaunchConfiguration('gripper_group'),
            'base_link': LaunchConfiguration('base_link'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
            'planner_id': LaunchConfiguration('planner_id'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'pre_grasp_offset_z': LaunchConfiguration('pre_grasp_offset_z'),
            'grasp_offset_z': LaunchConfiguration('grasp_offset_z'),
            'lift_height': LaunchConfiguration('lift_height'),
            'place_position': LaunchConfiguration('place_position'),
        }],
        remappings=[
            ('/target_pose', '/target_pose'),  # 物体位置话题
            ('/grasp_state', '/grasp_state'),  # 抓取状态话题
        ]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=PythonExpression([
            "'启动抓取规划节点 - 规划组: ' + '", LaunchConfiguration('arm_group'), "'"
        ])
    )
    
    return LaunchDescription([
        arm_group_arg,
        gripper_group_arg,
        base_link_arg,
        end_effector_link_arg,
        planner_id_arg,
        max_velocity_arg,
        max_acceleration_arg,
        pre_grasp_offset_z_arg,
        grasp_offset_z_arg,
        lift_height_arg,
        place_position_arg,
        launch_info,
        grasp_planner_node,
    ])

