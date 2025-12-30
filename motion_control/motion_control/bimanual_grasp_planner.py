#!/usr/bin/env python3
"""
成员C：双臂协调抓取规划与控制节点
实现双臂协作抓取功能
"""

import math
import csv
import json
import time
from datetime import datetime
from enum import Enum
from pathlib import Path
from threading import Thread
from typing import Optional, List, Dict

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 尝试导入pymoveit2
try:
    from pymoveit2 import GripperInterface, MoveIt2
    PYMOVEIT2_AVAILABLE = True
except ImportError:
    try:
        from motion_control.moveit2_simple import MoveIt2Simple as MoveIt2, GripperInterfaceSimple as GripperInterface
        PYMOVEIT2_AVAILABLE = False
    except ImportError:
        MoveIt2 = None
        GripperInterface = None
        PYMOVEIT2_AVAILABLE = False
        print("Warning: pymoveit2 not found, bimanual planning unavailable")


class BimanualGraspState(Enum):
    """双臂抓取状态机状态"""
    IDLE = "idle"
    PLANNING = "planning"
    LEFT_PRE_GRASP = "left_pre_grasp"
    RIGHT_PRE_GRASP = "right_pre_grasp"
    MOVING_TO_GRASP = "moving_to_grasp"  # 从预抓取移动到抓取位置
    GRASPING = "grasping"
    LIFTING = "lifting"
    MOVING_TO_PLACE = "moving_to_place"
    PLACING = "placing"
    RETURNING = "returning"
    COMPLETED = "completed"
    FAILED = "failed"


class BimanualGraspPlanner(Node):
    """双臂协调抓取规划与控制节点"""
    
    def __init__(self):
        super().__init__('bimanual_grasp_planner')
        
        # 声明参数
        self.declare_parameter('use_bimanual', True)
        self.declare_parameter('base_link', 'base')
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')  # 基于 openarm_moveit_config/config/ompl_planning.yaml
        self.declare_parameter('max_velocity', 0.3)  # 基于 openarm_moveit_config/config/joint_limits.yaml
        self.declare_parameter('max_acceleration', 0.3)
        self.declare_parameter('coordination_mode', 'sequential')  # sequential or parallel
        self.declare_parameter('namespace', '')  # 命名空间（用于 openarm.bimanual.launch.py 的命名空间配置）
        self.declare_parameter('fallback_to_single_arm', True)  # 不稳定时先单臂
        
        # 获取参数
        self.use_bimanual = self.get_parameter('use_bimanual').get_parameter_value().bool_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.coordination_mode = self.get_parameter('coordination_mode').get_parameter_value().string_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.fallback_to_single_arm = self.get_parameter('fallback_to_single_arm').get_parameter_value().bool_value
        
        # 关节名称
        self.left_joint_names = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        self.right_joint_names = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 初始化MoveIt2接口
        if MoveIt2 is not None:
            # 左臂
            self.left_moveit2 = MoveIt2(
                node=self,
                joint_names=self.left_joint_names,
                base_link_name=self.base_link,
                end_effector_name='openarm_left_link7',
                group_name='left_arm',
                callback_group=self.callback_group,
                use_move_group_action=True,
            )
            self.left_moveit2.planner_id = self.planner_id
            self.left_moveit2.max_velocity = self.max_velocity
            self.left_moveit2.max_acceleration = self.max_acceleration
            
            # 右臂
            self.right_moveit2 = MoveIt2(
                node=self,
                joint_names=self.right_joint_names,
                base_link_name=self.base_link,
                end_effector_name='openarm_right_link7',
                group_name='right_arm',
                callback_group=self.callback_group,
                use_move_group_action=True,
            )
            self.right_moveit2.planner_id = self.planner_id
            self.right_moveit2.max_velocity = self.max_velocity
            self.right_moveit2.max_acceleration = self.max_acceleration
        else:
            self.left_moveit2 = None
            self.right_moveit2 = None
            self.get_logger().error("MoveIt2 not available! Please install pymoveit2.")
        
        # 状态变量
        self.current_state = BimanualGraspState.IDLE
        self.target_pose: Optional[Point] = None
        self.selected_arm: Optional[str] = None  # 'left' 或 'right'，记录当前使用的臂
        
        # 订阅物体位置
        self.target_pose_sub = self.create_subscription(
            Point,
            '/target_pose',
            self.target_pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 发布状态
        from std_msgs.msg import String
        self.state_pub = self.create_publisher(String, '/bimanual_grasp_state', 10)
        
        # 状态机定时器
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_callback)
        
        # 根据命名空间设置话题前缀
        topic_prefix = f'/{self.namespace}' if self.namespace else ''
        self.get_logger().info(
            f'双臂抓取规划节点已启动 (命名空间: {self.namespace if self.namespace else "无"}, '
            f'协调模式: {self.coordination_mode}, 规划器: {self.planner_id})'
        )
        if self.fallback_to_single_arm:
            self.get_logger().info('已启用单臂回退模式（不稳定时先单臂）')
    
    def target_pose_callback(self, msg: Point):
        """接收物体位置"""
        if self.current_state == BimanualGraspState.IDLE:
            self.target_pose = msg
            self.get_logger().info(f'收到物体位置: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')
            self.current_state = BimanualGraspState.PLANNING
            self.publish_state()
    
    def state_machine_callback(self):
        """状态机主循环"""
        if self.left_moveit2 is None or self.right_moveit2 is None:
            return
        
        # 检查执行状态
        left_executing = self.left_moveit2.query_state().name == "EXECUTING"
        right_executing = self.right_moveit2.query_state().name == "EXECUTING"
        
        if left_executing or right_executing:
            return  # 正在执行，等待完成
        
        # 根据状态执行动作
        if self.current_state == BimanualGraspState.PLANNING:
            self.execute_planning()
        elif self.current_state == BimanualGraspState.LEFT_PRE_GRASP:
            self.execute_left_pre_grasp()
        elif self.current_state == BimanualGraspState.RIGHT_PRE_GRASP:
            self.execute_right_pre_grasp()
        elif self.current_state == BimanualGraspState.MOVING_TO_GRASP:
            self.execute_move_to_grasp()
        elif self.current_state == BimanualGraspState.GRASPING:
            self.execute_grasp()
        elif self.current_state == BimanualGraspState.LIFTING:
            self.execute_lift()
        elif self.current_state == BimanualGraspState.MOVING_TO_PLACE:
            self.execute_move_to_place()
        elif self.current_state == BimanualGraspState.PLACING:
            self.execute_place()
        elif self.current_state == BimanualGraspState.RETURNING:
            self.execute_return()
    
    def execute_planning(self):
        """执行规划：计算双臂抓取策略"""
        if self.target_pose is None:
            self.current_state = BimanualGraspState.FAILED
            return
        
        # 根据物体位置决定使用哪只手臂
        # 如果不稳定，优先使用单臂模式
        if self.fallback_to_single_arm:
            # 简单策略：根据y坐标选择
            if self.target_pose.y > 0:
                self.get_logger().info('选择左臂进行抓取（单臂模式）')
                self.selected_arm = 'left'
                self.current_state = BimanualGraspState.LEFT_PRE_GRASP
            else:
                self.get_logger().info('选择右臂进行抓取（单臂模式）')
                self.selected_arm = 'right'
                self.current_state = BimanualGraspState.RIGHT_PRE_GRASP
        else:
            # 双臂协调模式（未来扩展）
            self.get_logger().info('双臂协调模式（待实现）')
            self.selected_arm = 'left'
            self.current_state = BimanualGraspState.LEFT_PRE_GRASP
        
        self.publish_state()
    
    def execute_left_pre_grasp(self):
        """执行左臂预抓取"""
        if self.target_pose is None:
            self.current_state = BimanualGraspState.FAILED
            return
        
        pre_grasp_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + 0.15  # 预抓取高度
        ]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # 垂直向下
        
        try:
            self.left_moveit2.move_to_pose(
                position=pre_grasp_position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            if self.left_moveit2.wait_until_executed(timeout_sec=30.0):
                # 先移动到抓取位置，再抓取
                self.current_state = BimanualGraspState.MOVING_TO_GRASP
                self.get_logger().info('左臂预抓取位置到达')
            else:
                self.current_state = BimanualGraspState.FAILED
        except Exception as e:
            self.get_logger().error(f'左臂预抓取失败: {str(e)}')
            self.current_state = BimanualGraspState.FAILED
        
        self.publish_state()
    
    def execute_right_pre_grasp(self):
        """执行右臂预抓取"""
        if self.target_pose is None:
            self.current_state = BimanualGraspState.FAILED
            return
        
        pre_grasp_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + 0.15  # 预抓取高度
        ]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # 垂直向下
        
        try:
            self.right_moveit2.move_to_pose(
                position=pre_grasp_position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            if self.right_moveit2.wait_until_executed(timeout_sec=30.0):
                # 先移动到抓取位置，再抓取
                self.current_state = BimanualGraspState.MOVING_TO_GRASP
                self.get_logger().info('右臂预抓取位置到达')
            else:
                self.current_state = BimanualGraspState.FAILED
        except Exception as e:
            self.get_logger().error(f'右臂预抓取失败: {str(e)}')
            self.current_state = BimanualGraspState.FAILED
        
        self.publish_state()
    
    def execute_move_to_grasp(self):
        """从预抓取位置移动到抓取位置"""
        if self.target_pose is None or self.selected_arm is None:
            self.current_state = BimanualGraspState.FAILED
            return
        
        grasp_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + 0.02  # 抓取位置
        ]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # 垂直向下
        
        moveit2 = self.left_moveit2 if self.selected_arm == 'left' else self.right_moveit2
        
        try:
            moveit2.move_to_pose(
                position=grasp_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,  # 使用笛卡尔路径，直线下降
                cartesian_max_step=0.01,
            )
            if moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = BimanualGraspState.GRASPING
                self.get_logger().info('抓取位置到达')
            else:
                self.current_state = BimanualGraspState.FAILED
        except Exception as e:
            self.get_logger().error(f'移动到抓取位置失败: {str(e)}')
            self.current_state = BimanualGraspState.FAILED
        
        self.publish_state()
    
    def execute_grasp(self):
        """执行抓取（夹爪闭合）"""
        self.get_logger().info('执行抓取（夹爪闭合）')
        # 这里应该调用夹爪控制，暂时直接进入提升状态
        self.current_state = BimanualGraspState.LIFTING
        self.publish_state()
    
    def execute_lift(self):
        """执行提升"""
        if self.target_pose is None:
            self.current_state = BimanualGraspState.FAILED
            return
        
        lift_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + 0.2  # 提升高度
        ]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]
        
        # 根据选择的臂选择moveit2
        if self.selected_arm is None:
            self.get_logger().error('未选择手臂，无法执行提升')
            self.current_state = BimanualGraspState.FAILED
            self.publish_state()
            return
        
        moveit2 = self.left_moveit2 if self.selected_arm == 'left' else self.right_moveit2
        
        try:
            moveit2.move_to_pose(
                position=lift_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
            )
            if moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = BimanualGraspState.MOVING_TO_PLACE
                self.get_logger().info('物体提升成功')
            else:
                self.current_state = BimanualGraspState.FAILED
        except Exception as e:
            self.get_logger().error(f'提升失败: {str(e)}')
            self.current_state = BimanualGraspState.FAILED
        
        self.publish_state()
    
    def execute_move_to_place(self):
        """移动到放置位置"""
        place_position = [0.5, 0.3, 0.3]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]
        
        if self.selected_arm is None:
            self.get_logger().error('未选择手臂，无法移动到放置位置')
            self.current_state = BimanualGraspState.FAILED
            self.publish_state()
            return
        
        moveit2 = self.left_moveit2 if self.selected_arm == 'left' else self.right_moveit2
        
        try:
            moveit2.move_to_pose(
                position=place_position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            if moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = BimanualGraspState.PLACING
            else:
                self.current_state = BimanualGraspState.FAILED
        except Exception as e:
            self.get_logger().error(f'移动到放置位置失败: {str(e)}')
            self.current_state = BimanualGraspState.FAILED
        
        self.publish_state()
    
    def execute_place(self):
        """执行放置（夹爪打开）"""
        self.get_logger().info('执行放置（夹爪打开）')
        self.current_state = BimanualGraspState.RETURNING
        self.publish_state()
    
    def execute_return(self):
        """返回初始位置"""
        home_positions = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        
        if self.selected_arm is None:
            self.get_logger().error('未选择手臂，无法返回初始位置')
            self.current_state = BimanualGraspState.FAILED
            self.publish_state()
            return
        
        moveit2 = self.left_moveit2 if self.selected_arm == 'left' else self.right_moveit2
        
        try:
            moveit2.move_to_configuration(home_positions)
            if moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = BimanualGraspState.COMPLETED
                self.get_logger().info('抓取任务完成')
                # 重置状态
                self.current_state = BimanualGraspState.IDLE
                self.target_pose = None
                self.selected_arm = None
            else:
                self.current_state = BimanualGraspState.FAILED
        except Exception as e:
            self.get_logger().error(f'返回初始位置失败: {str(e)}')
            self.current_state = BimanualGraspState.FAILED
        
        self.publish_state()
    
    def publish_state(self):
        """发布状态"""
        from std_msgs.msg import String
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BimanualGraspPlanner()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    node.create_rate(1.0).sleep()
    
    try:
        while rclpy.ok():
            node.create_rate(1.0).sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()

