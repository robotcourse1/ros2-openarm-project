#!/usr/bin/env python3
"""
OpenArm双臂机器人启动文件
集成MoveIt2、控制器和TF发布
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
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
    
    left_can_interface_arg = DeclareLaunchArgument(
        'left_can_interface',
        default_value='can1',
        description='左臂CAN接口'
    )
    
    right_can_interface_arg = DeclareLaunchArgument(
        'right_can_interface',
        default_value='can0',
        description='右臂CAN接口'
    )
    
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='是否启动MoveIt2'
    )
    
    moveit_config_package_arg = DeclareLaunchArgument(
        'moveit_config_package',
        default_value='openarm_moveit_config',
        description='MoveIt配置包名称'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')
    use_moveit = LaunchConfiguration('use_moveit')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    
    # Robot State Publisher - 发布机器人TF
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
                ' bimanual:=true',
                ' ros2_control:=true',
                ' use_fake_hardware:=', use_fake_hardware,
                ' left_can_interface:=', left_can_interface,
                ' right_can_interface:=', right_can_interface,
                ' hand:=true'
            ])
        }]
    )
    
    # Joint State Publisher - 发布关节状态（用于仿真）
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
    
    # Controller Manager - ROS2 Control控制器管理器
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
                ' bimanual:=true',
                ' ros2_control:=true',
                ' use_fake_hardware:=', use_fake_hardware,
                ' left_can_interface:=', left_can_interface,
                ' right_can_interface:=', right_can_interface,
                ' hand:=true'
            ])
        }]
    )
    
    # 左臂控制器spawner
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='left_arm_controller_spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 右臂控制器spawner
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='right_arm_controller_spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 左臂夹爪控制器spawner
    left_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='left_hand_controller_spawner',
        arguments=['left_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 右臂夹爪控制器spawner
    right_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='right_hand_controller_spawner',
        arguments=['right_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # MoveIt2 Move Group - 如果启用MoveIt
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'config/moveit_controllers.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'config/trajectory_execution.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'config/planning_scene_monitor.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'config/kinematics.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'config/joint_limits.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
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
                    ' bimanual:=true',
                    ' ros2_control:=true',
                    ' use_fake_hardware:=', use_fake_hardware,
                    ' hand:=true'
                ])
            }
        ],
        condition=IfCondition(use_moveit)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_fake_hardware_arg,
        left_can_interface_arg,
        right_can_interface_arg,
        use_moveit_arg,
        moveit_config_package_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        controller_manager_node,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        left_hand_controller_spawner,
        right_hand_controller_spawner,
        move_group_node,
    ])

