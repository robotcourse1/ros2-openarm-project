from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    # 只使用 mujoco_ros_bridge 的内置可视化窗口，不再启动独立的 viewer
    mujoco_bridge = Node(
        package="openarm_env_bringup",
        executable="mujoco_ros_bridge",
        name="mujoco_ros_bridge",
        output="screen",
        parameters=[{
            "xml": xml_path,
            "enable_viewer": True,
            "drive_gripper": True,  # 启用夹爪控制
            "arm_command_alpha": 0.25,
            "arm_max_step": 0.10,
        }],
    )

    return [mujoco_bridge]


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


