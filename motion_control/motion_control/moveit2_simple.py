#!/usr/bin/env python3
"""
简化版MoveIt2接口
直接使用MoveIt2的ROS2服务和动作，不依赖pymoveit2
用于替代pymoveit2，实现基本运动规划功能
"""

import threading
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class MoveIt2State(Enum):
    """MoveIt2执行状态"""
    IDLE = 0
    REQUESTING = 1
    EXECUTING = 2


class MoveIt2Simple:
    """
    简化版MoveIt2接口
    直接使用MoveIt2的ROS2服务和动作
    """
    
    def __init__(
        self,
        node: Node,
        joint_names: List[str],
        base_link_name: str,
        end_effector_name: str,
        group_name: str = "arm",
        callback_group: Optional[CallbackGroup] = None,
    ):
        """
        初始化MoveIt2简化接口
        """
        self._node = node
        self._callback_group = callback_group
        self.__joint_names = joint_names
        self.__base_link_name = base_link_name
        self.__end_effector_name = end_effector_name
        self.__group_name = group_name
        
        # 状态
        self.__is_executing = False
        self.__execution_mutex = threading.Lock()
        self.__joint_state = None
        
        # 创建MoveGroup动作客户端
        self.__move_action_client = ActionClient(
            node=self._node,
            action_type=MoveGroup,
            action_name="move_action",
            goal_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )
        
        # 创建规划服务客户端
        self._plan_service = self._node.create_client(
            srv_type=GetMotionPlan,
            srv_name="plan_kinematic_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        
        # 订阅关节状态
        self._node.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )
        
        # 规划器ID和参数
        self.planner_id = "RRTConnectkConfigDefault"
        self.max_velocity = 0.3
        self.max_acceleration = 0.3
        
        self._node.get_logger().info(f"MoveIt2Simple初始化完成 (规划组: {group_name})")
    
    def _joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        with self.__execution_mutex:
            self.__joint_state = msg
    
    def query_state(self) -> MoveIt2State:
        """查询当前状态"""
        with self.__execution_mutex:
            if self.__is_executing:
                return MoveIt2State.EXECUTING
            else:
                return MoveIt2State.IDLE
    
    def wait_until_executed(self, timeout_sec: float = 10.0) -> bool:
        """等待执行完成"""
        # 简化实现：等待一段时间
        import time
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.query_state() == MoveIt2State.IDLE:
                return True
            time.sleep(0.1)
        return False
    
    def move_to_pose(
        self,
        position: Optional[List[float]] = None,
        quat_xyzw: Optional[List[float]] = None,
        cartesian: bool = False,
        **kwargs
    ):
        """移动到目标姿态"""
        if not self.__move_action_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error("MoveIt2动作服务器未就绪")
            return False
        
        # 创建目标
        goal = MoveGroup.Goal()
        goal.request.group_name = self.__group_name
        goal.request.planner_id = self.planner_id
        goal.request.max_velocity_scaling_factor = self.max_velocity
        goal.request.max_acceleration_scaling_factor = self.max_acceleration
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        
        # 设置目标约束
        goal.request.goal_constraints = [Constraints()]
        
        if position is not None:
            # 位置约束
            from moveit_msgs.msg import PositionConstraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = self.__base_link_name
            pos_constraint.link_name = self.__end_effector_name
            pos_constraint.constraint_region.primitive_poses.append(
                Pose(position=Point(x=position[0], y=position[1], z=position[2]))
            )
            goal.request.goal_constraints[0].position_constraints.append(pos_constraint)
        
        if quat_xyzw is not None:
            # 姿态约束
            from moveit_msgs.msg import OrientationConstraint
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = self.__base_link_name
            ori_constraint.link_name = self.__end_effector_name
            ori_constraint.orientation = Quaternion(
                x=quat_xyzw[0], y=quat_xyzw[1], z=quat_xyzw[2], w=quat_xyzw[3]
            )
            ori_constraint.absolute_x_axis_tolerance = 0.1
            ori_constraint.absolute_y_axis_tolerance = 0.1
            ori_constraint.absolute_z_axis_tolerance = 0.1
            goal.request.goal_constraints[0].orientation_constraints.append(ori_constraint)
        
        # 发送目标
        with self.__execution_mutex:
            self.__is_executing = True
        
        future = self.__move_action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def _goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error("目标被拒绝")
            with self.__execution_mutex:
                self.__is_executing = False
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """结果回调"""
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self._node.get_logger().info("运动执行成功")
        else:
            self._node.get_logger().warn(f"运动执行失败，错误码: {result.error_code.val}")
        
        with self.__execution_mutex:
            self.__is_executing = False
    
    def move_to_configuration(self, joint_positions: List[float], **kwargs):
        """移动到关节配置"""
        if not self.__move_action_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error("MoveIt2动作服务器未就绪")
            return False
        
        # 创建目标
        goal = MoveGroup.Goal()
        goal.request.group_name = self.__group_name
        goal.request.planner_id = self.planner_id
        goal.request.max_velocity_scaling_factor = self.max_velocity
        goal.request.max_acceleration_scaling_factor = self.max_acceleration
        
        # 设置关节约束
        goal.request.goal_constraints = [Constraints()]
        for i, joint_name in enumerate(self.__joint_names):
            if i < len(joint_positions):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = joint_positions[i]
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                goal.request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # 发送目标
        with self.__execution_mutex:
            self.__is_executing = True
        
        future = self.__move_action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def plan(
        self,
        position: Optional[List[float]] = None,
        quat_xyzw: Optional[List[float]] = None,
        **kwargs
    ) -> Optional[JointTrajectory]:
        """规划轨迹"""
        if not self._plan_service.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().error("规划服务未就绪")
            return None
        
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.__group_name
        request.motion_plan_request.planner_id = self.planner_id
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0
        
        # 设置目标约束（类似move_to_pose）
        # ... 实现类似move_to_pose的逻辑
        
        future = self._plan_service.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        
        if future.result() and future.result().motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            return future.result().motion_plan_response.trajectory.joint_trajectory
        return None
    
    def execute(self, trajectory: JointTrajectory) -> bool:
        """执行轨迹"""
        if not self.__move_action_client.wait_for_server(timeout_sec=2.0):
            return False
        
        # 使用ExecuteTrajectory动作
        from rclpy.action import ActionClient
        execute_client = ActionClient(
            self._node,
            ExecuteTrajectory,
            "execute_trajectory",
            callback_group=self._callback_group,
        )
        
        if not execute_client.wait_for_server(timeout_sec=2.0):
            return False
        
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        with self.__execution_mutex:
            self.__is_executing = True
        
        future = execute_client.send_goal_async(goal)
        # 简化处理...
        
        return True


class GripperInterfaceSimple:
    """简化版夹爪接口"""
    
    def __init__(
        self,
        node: Node,
        gripper_joint_names: List[str],
        open_gripper_joint_positions: List[float],
        closed_gripper_joint_positions: List[float],
        **kwargs
    ):
        self._node = node
        self.gripper_joint_names = gripper_joint_names
        self.open_positions = open_gripper_joint_positions
        self.closed_positions = closed_gripper_joint_positions
        self.is_open = True
    
    def open(self, **kwargs):
        """打开夹爪"""
        self._node.get_logger().info("打开夹爪（简化实现）")
        self.is_open = True
    
    def close(self, **kwargs):
        """闭合夹爪"""
        self._node.get_logger().info("闭合夹爪（简化实现）")
        self.is_open = False
    
    def wait_until_executed(self, timeout_sec: float = 5.0) -> bool:
        """等待执行完成"""
        return True

