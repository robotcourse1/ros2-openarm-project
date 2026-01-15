#!/usr/bin/env python3
"""
MuJoCo → ROS 2 bridge for the OpenArm environment.

Enhanced to support joint command subscription and visualization.

Topics:
  - /mujoco_joint_states (sensor_msgs/JointState) - published
  - /joint_command (sensor_msgs/JointState) - subscribed
"""

from __future__ import annotations

import pathlib
import time
from typing import Dict, List
import threading

import mujoco
import mujoco.viewer
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

from openarm_env_bringup.mujoco_xml_utils import patch_xml_mesh_paths

from geometry_msgs.msg import PoseStamped


def resolve_xml_path(param_value: str | None) -> pathlib.Path:
    """Resolve MuJoCo XML path, falling back to package share."""
    if param_value:
        return pathlib.Path(param_value).resolve()
    pkg_share = pathlib.Path(get_package_share_directory("openarm_env_description"))
    return pkg_share / "mujoco" / "openarm_env.xml"


def _mat_to_quat_wxyz(m: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to quaternion [w, x, y, z]."""
    # Robust matrix->quat conversion (assumes m is a proper rotation)
    t = float(np.trace(m))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    else:
        if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    n = np.linalg.norm(q)
    return q if n == 0.0 else (q / n)


class MujocoRosBridge(Node):
    """Enhanced MuJoCo → ROS 2 bridge with visualization and command subscription."""

    def __init__(self) -> None:
        super().__init__("mujoco_ros_bridge")

        self.declare_parameter("xml", "")
        self.declare_parameter("enable_viewer", True)

        # Control smoothing / stability knobs
        self.declare_parameter("arm_command_alpha", 0.08)  # 0..1, higher = follow commands faster (降低以获得更平滑运动)
        self.declare_parameter("arm_max_step", 0.02)  # rad / update (100Hz) (降低以限制最大速度)
        self.declare_parameter("arm_hold_initial_pose", True)

        # Allow disabling gripper actuation from the bridge to avoid conflicts
        self.declare_parameter("drive_gripper", True)  # 启用夹爪控制
        self._drive_gripper = bool(self.get_parameter("drive_gripper").get_parameter_value().bool_value)

        xml_param = self.get_parameter("xml").get_parameter_value().string_value
        self.enable_viewer = self.get_parameter("enable_viewer").get_parameter_value().bool_value

        self._arm_command_alpha = float(
            self.get_parameter("arm_command_alpha").get_parameter_value().double_value
        )
        self._arm_max_step = float(self.get_parameter("arm_max_step").get_parameter_value().double_value)
        self._arm_hold_initial_pose = bool(
            self.get_parameter("arm_hold_initial_pose").get_parameter_value().bool_value
        )

        # Clamp parameters to sane ranges
        if not (0.0 <= self._arm_command_alpha <= 1.0):
            self.get_logger().warn("arm_command_alpha out of range, clamping to [0,1]")
            self._arm_command_alpha = max(0.0, min(1.0, self._arm_command_alpha))
        if self._arm_max_step <= 0.0:
            self.get_logger().warn("arm_max_step must be > 0, defaulting to 0.1")
            self._arm_max_step = 0.1

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

        # Guard all accesses to self._data (mj_step, viewer, renderer)
        self._mj_lock = threading.RLock()

        # Pre-compute mapping from joint names to qpos indices.
        self._joint_names: List[str] = []
        self._joint_qpos_index: Dict[str, int] = {}

        # Match the joint names in openarm_env_description/mujoco/openarm_env.xml
        joint_name_candidates = [
            *(f"openarm_left_joint{i}" for i in range(1, 8)),
            *(f"openarm_right_joint{i}" for i in range(1, 8)),
        ]

        # Map joint names to actuator IDs for control
        self._joint_actuator_ids: Dict[str, int] = {}
        for i in range(1, 8):
            # Left arm actuators (named left_motor1...7)
            j_name = f"openarm_left_joint{i}"
            act_name = f"left_motor{i}"
            act_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if act_id != -1:
                self._joint_actuator_ids[j_name] = act_id
            
            # Right arm actuators (named right_motor1...7)
            j_name = f"openarm_right_joint{i}"
            act_name = f"right_motor{i}"
            act_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if act_id != -1:
                self._joint_actuator_ids[j_name] = act_id

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

        # 添加夹爪关节映射
        self._gripper_joint_names: List[str] = []
        self._gripper_qpos_index: Dict[str, int] = {}
        self._gripper_actuator_ids: Dict[str, int] = {}
        
        gripper_map = {
            "openarm_left_gripper_joint": "left_gripper_main",
            "openarm_left_gripper_joint_mirror": "left_gripper_mirror",
            "openarm_right_gripper_joint": "right_gripper_main", 
            "openarm_right_gripper_joint_mirror": "right_gripper_mirror"
        }
        
        for name in gripper_map.keys():
            try:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                qpos_index = self._model.jnt_qposadr[jid]
                self._gripper_joint_names.append(name)
                self._gripper_qpos_index[name] = int(qpos_index)
                
                # Get actuator ID
                act_name = gripper_map[name]
                act_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                if act_id != -1:
                    self._gripper_actuator_ids[name] = act_id
            except:
                pass

        # Store target positions for smooth control
        self._target_positions: Dict[str, float] = {name: 0.0 for name in self._joint_names}

        # A separate internal "filtered" target (what we actually send to actuators)
        self._filtered_positions: Dict[str, float] = {name: 0.0 for name in self._joint_names}

        # 夹爪状态 (0.0 = 闭合, 0.04 = 打开)
        self._gripper_open = True
        self._gripper_target = 0.04  # 默认打开

        # Initialize targets to the current sim pose to avoid snapping/oscillation at startup
        if self._arm_hold_initial_pose and self._joint_names:
            for n in self._joint_names:
                q = float(self._data.qpos[self._joint_qpos_index[n]])
                self._target_positions[n] = q
                self._filtered_positions[n] = q
            self.get_logger().info("Initialized arm targets from current MuJoCo qpos (hold initial pose)")

        # 相机设置
        self._bridge = CvBridge()
        self._camera_id = None
        
        # 查找 depth_camera
        try:
            self._camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, "depth_camera")
            self.get_logger().info(f"📷 找到相机: depth_camera (ID={self._camera_id})")
        except:
            self.get_logger().warn("⚠️ 未找到 depth_camera，相机功能将被禁用")
        
        # 相机渲染器
        self._renderer = None
        if self._camera_id is not None:
            self._renderer = mujoco.Renderer(self._model, height=480, width=640)
            self.get_logger().info("🎥 相机渲染器已初始化 (640x480)")

        # Publishers and Subscribers
        self._pub_joint = self.create_publisher(JointState, "/mujoco_joint_states", 10)
        
        # 相机图像发布器
        if self._camera_id is not None:
            self._pub_camera_rgb = self.create_publisher(Image, "/camera/image_raw", 10)
            self._pub_camera_depth = self.create_publisher(Image, "/camera/depth/image_raw", 10)
            self._pub_camera_info = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
            self.get_logger().info("📡 相机话题已创建")
        
        self._sub_command = self.create_subscription(
            JointState, "/joint_command", self._command_callback, 10
        )
        
        # 订阅夹爪命令
        self._sub_gripper = self.create_subscription(
            String, "/gripper_command", self._gripper_callback, 10
        )

        # 启动可视化窗口（在单独的线程中）
        self._viewer = None
        self._viewer_running = False
        if self.enable_viewer:
            self._viewer_running = True
            self._viewer_thread = threading.Thread(target=self._run_viewer, daemon=True)
            self._viewer_thread.start()
            self.get_logger().info("🎬 MuJoCo 可视化窗口已启动!")

        # Timer for stepping the simulation and publishing joint states.
        self._last_time = time.time()
        self._timer = self.create_timer(0.01, self._on_timer)  # 100 Hz
        
        # 相机图像发布定时器（10 Hz，降低频率以提高性能）
        if self._camera_id is not None:
            self._camera_timer = self.create_timer(0.1, self._publish_camera_images)
            self.get_logger().info("📸 相机图像发布器已启动 (10 Hz)")

        self.get_logger().info(
            f"mujoco_ros_bridge started with joints: {', '.join(self._joint_names) or '(none)'}"
        )

        # === Ground-truth EE pose publishers (world frame from MuJoCo) ===
        # We publish the pose of link7 bodies for both arms in MuJoCo world coordinates.
        # MuJoCo world frame is the same as the XML's <worldbody> frame.
        self._pub_left_ee_pose = self.create_publisher(PoseStamped, "/left_ee_pose_world", 10)
        self._pub_right_ee_pose = self.create_publisher(PoseStamped, "/right_ee_pose_world", 10)

        self._left_ee_body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "openarm_left_link7")
        self._right_ee_body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "openarm_right_link7")

        if self._left_ee_body_id == -1:
            self.get_logger().warn("EE body not found: openarm_left_link7; /left_ee_pose_world will not publish")
        if self._right_ee_body_id == -1:
            self.get_logger().warn("EE body not found: openarm_right_link7; /right_ee_pose_world will not publish")

    def _run_viewer(self):
        """在单独的线程中运行可视化窗口"""
        try:
            # NOTE: viewer reads mjData; we must serialize access with the sim thread.
            with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
                self._viewer = viewer
                self.get_logger().info("📺 可视化窗口已打开，按 ESC 键关闭")

                while self._viewer_running and viewer.is_running():
                    # Only sync while holding the mutex to avoid mjData stack corruption.
                    with self._mj_lock:
                        viewer.sync()
                    time.sleep(0.01)

        except Exception as e:
            self.get_logger().error(f"可视化窗口错误: {e}")

    def _command_callback(self, msg: JointState) -> None:
        """Receive joint commands and update target positions."""
        for i, name in enumerate(msg.name):
            if name in self._target_positions and i < len(msg.position):
                self._target_positions[name] = msg.position[i]
                self.get_logger().info(f"✅ Command received: {name} -> {msg.position[i]:.3f}")
            else:
                self.get_logger().warn(f"⚠️ Unknown joint name or invalid index: {name}")

    def _gripper_callback(self, msg: String) -> None:
        """Receive gripper commands and update target positions."""
        if msg.data.lower() == "open":
            self._gripper_open = True
            self._gripper_target = 0.04
            self.get_logger().info("✅ Gripper command: OPEN")
        elif msg.data.lower() == "close":
            self._gripper_open = False
            self._gripper_target = 0.0
            self.get_logger().info("✅ Gripper command: CLOSE")
        else:
            self.get_logger().warn(f"⚠️ Unknown gripper command: {msg.data}")

    def _on_timer(self) -> None:
        """Step the MuJoCo simulation and publish JointState."""

        with self._mj_lock:
            # 1. Arm joints (smoothed + limited step)
            alpha = self._arm_command_alpha
            max_step = self._arm_max_step
            for name, act_id in self._joint_actuator_ids.items():
                if name not in self._target_positions:
                    continue

                desired = float(self._target_positions[name])
                current = float(self._filtered_positions.get(name, desired))

                filtered = current + alpha * (desired - current)

                delta = filtered - current
                if delta > max_step:
                    filtered = current + max_step
                elif delta < -max_step:
                    filtered = current - max_step

                self._filtered_positions[name] = filtered
                self._data.ctrl[act_id] = filtered

            # 2. Gripper joints (optional)
            if self._drive_gripper:
                for _name, act_id in self._gripper_actuator_ids.items():
                    self._data.ctrl[act_id] = float(self._gripper_target)

            # 实时同步：计算应该步进多少次
            current_time = time.time()
            elapsed = current_time - self._last_time
            self._last_time = current_time
            
            # 根据实际时间流逝计算需要的仿真步数
            # timestep=0.001秒，timer=0.01秒，理论上每次应该执行10步
            # 但为了更平滑，我们固定每次执行5步（相当于0.005秒仿真时间）
            # 这样仿真速度约为实时的50%，可以清楚看到运动过程
            steps = 5
            
            # Step the simulation forward in time
            for _ in range(steps):
                mujoco.mj_step(self._model, self._data)

            # Detect NaNs early; once NaNs appear, MuJoCo viewer/camera frequently segfaults.
            if (not np.isfinite(self._data.qpos).all()) or (not np.isfinite(self._data.qvel).all()):
                self.get_logger().error(
                    "Detected NaN/Inf in MuJoCo state (qpos/qvel). Stopping viewer and shutting down node."
                )
                self._viewer_running = False
                # Let destroy_node close viewer gracefully
                rclpy.shutdown()
                return

            # Publish joint states (include gripper joints too)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            all_names = list(self._joint_names) + list(self._gripper_joint_names)
            msg.name = all_names

            positions: List[float] = []
            for n in self._joint_names:
                positions.append(float(self._data.qpos[self._joint_qpos_index[n]]))
            for n in self._gripper_joint_names:
                positions.append(float(self._data.qpos[self._gripper_qpos_index[n]]))
            msg.position = positions
            msg.velocity = []

            # Ground-truth EE pose (MuJoCo body pose in world frame)
            stamp = msg.header.stamp
            if self._left_ee_body_id != -1:
                p = self._data.xpos[self._left_ee_body_id].copy()
                r = self._data.xmat[self._left_ee_body_id].reshape(3, 3).copy()
                q = _mat_to_quat_wxyz(r)
                ee = PoseStamped()
                ee.header.stamp = stamp
                ee.header.frame_id = "world"
                ee.pose.position.x = float(p[0])
                ee.pose.position.y = float(p[1])
                ee.pose.position.z = float(p[2])
                ee.pose.orientation.w = float(q[0])
                ee.pose.orientation.x = float(q[1])
                ee.pose.orientation.y = float(q[2])
                ee.pose.orientation.z = float(q[3])
                self._pub_left_ee_pose.publish(ee)

            if self._right_ee_body_id != -1:
                p = self._data.xpos[self._right_ee_body_id].copy()
                r = self._data.xmat[self._right_ee_body_id].reshape(3, 3).copy()
                q = _mat_to_quat_wxyz(r)
                ee = PoseStamped()
                ee.header.stamp = stamp
                ee.header.frame_id = "world"
                ee.pose.position.x = float(p[0])
                ee.pose.position.y = float(p[1])
                ee.pose.position.z = float(p[2])
                ee.pose.orientation.w = float(q[0])
                ee.pose.orientation.x = float(q[1])
                ee.pose.orientation.y = float(q[2])
                ee.pose.orientation.z = float(q[3])
                self._pub_right_ee_pose.publish(ee)

        # Publish outside the lock
        self._pub_joint.publish(msg)

    def _publish_camera_images(self) -> None:
        """发布相机RGB和深度图像"""
        if self._camera_id is None or self._renderer is None:
            return

        try:
            with self._mj_lock:
                # 更新渲染器
                self._renderer.update_scene(self._data, camera=self._camera_id)

                # 渲染RGB图像
                rgb_image = self._renderer.render()

                # 渲染深度图像
                self._renderer.enable_depth_rendering()
                depth_image = self._renderer.render()

            # 转换为ROS Image消息 (no need to hold MuJoCo lock)
            rgb_msg = self._bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "depth_camera_optical_frame"
            self._pub_camera_rgb.publish(rgb_msg)

            depth_array = depth_image[:, :, 0].astype(np.float32)
            depth_msg = self._bridge.cv2_to_imgmsg(depth_array, encoding="32FC1")
            depth_msg.header.stamp = rgb_msg.header.stamp
            depth_msg.header.frame_id = "depth_camera_optical_frame"
            self._pub_camera_depth.publish(depth_msg)

            # CameraInfo
            camera_info_msg = CameraInfo()
            camera_info_msg.header = rgb_msg.header
            camera_info_msg.height = 480
            camera_info_msg.width = 640

            fx = fy = 640 / (2 * np.tan(np.radians(45) / 2))
            cx = 640 / 2
            cy = 480 / 2

            camera_info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]

            self._pub_camera_info.publish(camera_info_msg)

        except Exception as e:
            self.get_logger().error(f"相机图像发布错误: {e}", throttle_duration_sec=1.0)

    def destroy_node(self):
        """清理资源"""
        self._viewer_running = False
        if self._viewer is not None:
            self._viewer.close()
        super().destroy_node()


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


