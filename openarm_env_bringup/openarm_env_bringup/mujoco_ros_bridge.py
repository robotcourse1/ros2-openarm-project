#!/usr/bin/env python3
"""
MuJoCo → ROS 2 bridge for the OpenArm environment.

Minimal goal: step the MuJoCo simulation defined in openarm_env.xml
and publish a JointState message so that downstream components
can verify data flow.

This node does NOT try to mirror the full OpenArm URDF; instead it
publishes the simplified MuJoCo arm joints defined in openarm_env.xml:
  - joint1
  - joint2
  - joint3

Topic:
  - /mujoco_joint_states (sensor_msgs/JointState)
"""

from __future__ import annotations

import pathlib
import time
from typing import Dict, List

import mujoco
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from openarm_env_bringup.mujoco_xml_utils import patch_xml_mesh_paths


def resolve_xml_path(param_value: str | None) -> pathlib.Path:
    """Resolve MuJoCo XML path, falling back to package share."""
    if param_value:
        return pathlib.Path(param_value).resolve()
    pkg_share = pathlib.Path(get_package_share_directory("openarm_env_description"))
    return pkg_share / "mujoco" / "openarm_env.xml"


class MujocoRosBridge(Node):
    """Minimal MuJoCo → ROS 2 bridge publishing JointState."""

    def __init__(self) -> None:
        super().__init__("mujoco_ros_bridge")

        self.declare_parameter("xml", "")
        xml_param = self.get_parameter("xml").get_parameter_value().string_value
        xml_path = resolve_xml_path(xml_param)

        if not xml_path.exists():
            raise FileNotFoundError(f"MuJoCo xml not found: {xml_path}")

        patched_xml_path: pathlib.Path | None = None
        try:
            patched_xml_path = patch_xml_mesh_paths(xml_path)
            self.get_logger().info(f"Using patched MuJoCo XML: {patched_xml_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not patch MuJoCo XML mesh paths: {e}")
            self.get_logger().warn("Falling back to original XML; mesh loads may fail.")

        self.get_logger().info(f"Loading MuJoCo model from: {patched_xml_path or xml_path}")
        self._model = mujoco.MjModel.from_xml_path(str(patched_xml_path or xml_path))
        self._data = mujoco.MjData(self._model)

        # Pre-compute mapping from joint names to qpos indices.
        # We use the simplified arm joint names defined in openarm_env.xml.
        self._joint_names: List[str] = []
        self._joint_qpos_index: Dict[str, int] = {}

        # Match the joint names in openarm_env_description/mujoco/openarm_env.xml
        joint_name_candidates = [
            *(f"openarm_left_joint{i}" for i in range(1, 8)),
            *(f"openarm_right_joint{i}" for i in range(1, 8)),
        ]

        for name in joint_name_candidates:
            try:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            except Exception:
                self.get_logger().warn(f"Joint '{name}' not found in MuJoCo model; skipping.")
                continue

            qpos_index = self._model.jnt_qposadr[jid]
            self._joint_names.append(name)
            self._joint_qpos_index[name] = int(qpos_index)

        if not self._joint_names:
            self.get_logger().warn(
                "No MuJoCo joints found for bridge; JointState messages will be empty."
            )

        self._pub_joint = self.create_publisher(JointState, "/mujoco_joint_states", 10)

        # Timer for stepping the simulation and publishing joint states.
        self._last_time = time.time()
        self._timer = self.create_timer(0.01, self._on_timer)  # 100 Hz

        self.get_logger().info(
            f"mujoco_ros_bridge started with joints: {', '.join(self._joint_names) or '(none)'}"
        )

    def _on_timer(self) -> None:
        """Step the MuJoCo simulation and publish JointState."""
        # Advance physics by one step (fixed timestep from the XML).
        mujoco.mj_step(self._model, self._data)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [float(self._data.qpos[self._joint_qpos_index[n]]) for n in self._joint_names]
        # Velocities and efforts are optional; leave empty for now.

        self._pub_joint.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MujocoRosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


