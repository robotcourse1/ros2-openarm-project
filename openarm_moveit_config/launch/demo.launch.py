#!/usr/bin/env python3
"""
OpenArm MoveIt2 Demo Launch文件
启动MoveIt2 move_group节点和RViz可视化
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
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
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('openarm_description'),
                    'urdf/robot/v10.urdf.xacro'
                ]),
                ' bimanual:=false',
                ' ros2_control:=true',
                ' use_fake_hardware:=', use_fake_hardware,
                ' hand:=true'
            ])
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/joint_states']
        }],
        condition=IfCondition(use_fake_hardware)
    )
    
    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('openarm_description'),
                    'urdf/robot/v10.urdf.xacro'
                ]),
                ' bimanual:=false',
                ' ros2_control:=true',
                ' use_fake_hardware:=', use_fake_hardware,
                ' hand:=true'
            ])
        }]
    )
    
    # Arm Controller Spawner (根据SRDF配置，可能是left_arm_controller或arm_controller)
    # 注意：这里需要根据实际的控制器名称调整
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Hand Controller Spawner (可选)
    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='hand_controller_spawner',
        arguments=['left_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # MoveIt2 Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('openarm_moveit_config'),
                'config/moveit_controllers.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('openarm_moveit_config'),
                'config/trajectory_execution.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('openarm_moveit_config'),
                'config/planning_scene_monitor.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('openarm_moveit_config'),
                'config/kinematics.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('openarm_moveit_config'),
                'config/joint_limits.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('openarm_moveit_config'),
                'config/ompl_planning.yaml'
            ]),
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': True,
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('openarm_description'),
                        'urdf/robot/v10.urdf.xacro'
                    ]),
                    ' bimanual:=false',
                    ' ros2_control:=true',
                    ' use_fake_hardware:=', use_fake_hardware,
                    ' hand:=true'
                ])
            }
        ],
    )
    
    # RViz (可选)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('openarm_description'),
            'rviz/arm_only.rviz'
        ])],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_fake_hardware_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        controller_manager_node,
        arm_controller_spawner,
        hand_controller_spawner,
        move_group_node,
        rviz_node,
    ])

