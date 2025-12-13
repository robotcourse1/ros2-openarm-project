from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xml_arg = DeclareLaunchArgument(
        "xml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("openarm_env_description"), "mujoco", "openarm_env.xml"]
        ),
        description="Path to MuJoCo XML scene file",
    )

    viewer_script = PathJoinSubstitution(
        [FindPackageShare("openarm_env_bringup"), "scripts", "view_mujoco_env.py"]
    )
    xml_path = LaunchConfiguration("xml")

    run_viewer = ExecuteProcess(
        cmd=["python3", viewer_script, xml_path],
        output="screen",
    )

    return LaunchDescription([xml_arg, run_viewer])




