from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _as_bool(s: str) -> bool:
    return s.strip().lower() in ("1", "true", "yes", "y", "on")


def _launch_setup(context, *args, **kwargs):
    static = _as_bool(LaunchConfiguration("static").perform(context))
    xml_override = LaunchConfiguration("xml").perform(context).strip()

    if static:
        xml_path = PathJoinSubstitution(
            [FindPackageShare("openarm_env_description"), "mujoco", "openarm_env_static.xml"]
        ).perform(context)
    elif xml_override:
        xml_path = xml_override
    else:
        xml_path = PathJoinSubstitution(
            [FindPackageShare("openarm_env_description"), "mujoco", "openarm_env.xml"]
        ).perform(context)

    viewer_script = PathJoinSubstitution(
        [FindPackageShare("openarm_env_bringup"), "scripts", "view_mujoco_env.py"]
    ).perform(context)

    run_viewer = ExecuteProcess(
        cmd=["python3", viewer_script, xml_path],
        output="screen",
    )

    mujoco_bridge = Node(
        package="openarm_env_bringup",
        executable="mujoco_ros_bridge",
        name="mujoco_ros_bridge",
        output="screen",
        parameters=[{"xml": xml_path}],
    )

    return [run_viewer, mujoco_bridge]


def generate_launch_description():
    static_arg = DeclareLaunchArgument(
        "static",
        default_value="true",
        description="If true, load openarm_env_static.xml (stable, URDF-like). If false, load dynamic scene.",
    )
    xml_arg = DeclareLaunchArgument(
        "xml",
        default_value="",
        description="Optional override path to MuJoCo XML scene file (used when static:=false)",
    )

    return LaunchDescription([static_arg, xml_arg, OpaqueFunction(function=_launch_setup)])


