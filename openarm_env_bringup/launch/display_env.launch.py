from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui'
    )
    
    description_package_arg = DeclareLaunchArgument(
        'description_package',
        default_value='openarm_env_description',
        description='Package with env xacro files'
    )
    
    description_file_arg = DeclareLaunchArgument(
        'description_file',
        default_value='urdf/openarm_with_env.xacro',
        description='Xacro file to load'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('openarm_env_bringup'),
            'rviz',
            'openarm_env.rviz'
        ]),
        description='RViz config file'
    )

    # Get configurations
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    use_gui = LaunchConfiguration('use_gui')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Xacro path
    xacro_path = PathJoinSubstitution([
        FindPackageShare(description_package),
        description_file
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_path]),
                value_type=str
            )
        }]
    )

    # Joint state publisher (always use GUI version for simplicity)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        use_gui_arg,
        description_package_arg,
        description_file_arg,
        rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
