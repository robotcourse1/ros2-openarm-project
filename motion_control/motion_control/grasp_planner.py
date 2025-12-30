#!/usr/bin/env python3
"""
成员C：运动规划与控制节点
实现基于MoveIt2的抓取规划与控制功能
基于 openarm_moveit_config 和 openarm_control/arm_controller.py
"""

import math
import os
import time
from enum import Enum
from threading import Thread
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# 尝试导入pymoveit2，如果不可用则使用备用方案
try:
    from pymoveit2 import GripperInterface, MoveIt2
except ImportError:
    MoveIt2 = None
    GripperInterface = None
    print("Warning: pymoveit2 not found, using fallback implementation")

# 尝试导入 openarm_control 的 arm_controller
try:
    import sys
    # 添加 openarm_control 到路径
    openarm_control_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'openarm_control', 'scripts'
    )
    if os.path.exists(openarm_control_path):
        sys.path.insert(0, openarm_control_path)
        from arm_controller import ArmController
        ARM_CONTROLLER_AVAILABLE = True
    else:
        ARM_CONTROLLER_AVAILABLE = False
        ArmController = None
except ImportError:
    ARM_CONTROLLER_AVAILABLE = False
    ArmController = None
    print("Warning: openarm_control/arm_controller.py not found, using MoveIt2 directly")


class GraspState(Enum):
    """抓取状态机状态"""
    IDLE = "idle"                    # 空闲状态
    MOVING_TO_PRE_GRASP = "moving_to_pre_grasp"  # 移动到预抓取位置
    MOVING_TO_GRASP = "moving_to_grasp"          # 移动到抓取位置
    GRASPING = "grasping"            # 抓取中（夹爪闭合）
    LIFTING = "lifting"              # 提升物体
    MOVING_TO_PLACE = "moving_to_place"  # 移动到放置位置
    PLACING = "placing"              # 放置物体
    RETURNING = "returning"          # 返回初始位置
    COMPLETED = "completed"           # 完成
    FAILED = "failed"                 # 失败


class GraspPlanner(Node):
    """抓取规划与控制节点"""
    
    def __init__(self):
        super().__init__('grasp_planner')
        
        # 声明参数
        self.declare_parameter('arm_group', 'left_arm')
        self.declare_parameter('gripper_group', 'left_hand')
        self.declare_parameter('base_link', 'base')
        self.declare_parameter('end_effector_link', 'openarm_left_link7')
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')  # 基于 openarm_moveit_config/config/ompl_planning.yaml
        self.declare_parameter('max_velocity', 0.3)  # 基于 openarm_moveit_config/config/joint_limits.yaml
        self.declare_parameter('max_acceleration', 0.3)
        self.declare_parameter('use_arm_controller', False)  # 是否使用 openarm_control/arm_controller.py
        self.declare_parameter('namespace', '')  # 命名空间（用于双臂配置）
        self.declare_parameter('pre_grasp_offset_z', 0.15)  # 预抓取位置在物体上方的高度
        self.declare_parameter('grasp_offset_z', 0.02)      # 抓取位置相对物体的高度偏移
        self.declare_parameter('lift_height', 0.2)           # 提升高度
        self.declare_parameter('place_position', [0.5, 0.3, 0.3])  # 放置位置
        
        # 获取参数
        self.arm_group = self.get_parameter('arm_group').get_parameter_value().string_value
        self.gripper_group = self.get_parameter('gripper_group').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter('end_effector_link').get_parameter_value().string_value
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.use_arm_controller = self.get_parameter('use_arm_controller').get_parameter_value().bool_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.pre_grasp_offset_z = self.get_parameter('pre_grasp_offset_z').get_parameter_value().double_value
        self.grasp_offset_z = self.get_parameter('grasp_offset_z').get_parameter_value().double_value
        self.lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self.place_position = self.get_parameter('place_position').get_parameter_value().double_array_value
        
        # 关节名称（根据openarm配置）
        if 'left' in self.arm_group:
            self.joint_names = [
                'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
                'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
                'openarm_left_joint7'
            ]
            self.gripper_joint_names = [
                'openarm_left_finger_joint1', 'openarm_left_finger_joint2'
            ]
        else:
            self.joint_names = [
                'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
                'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
                'openarm_right_joint7'
            ]
            self.gripper_joint_names = [
                'openarm_right_finger_joint1', 'openarm_right_finger_joint2'
            ]
        
        # 夹爪开合位置（根据SRDF配置）
        self.open_gripper_positions = [0.03, 0.03]  # 打开
        self.closed_gripper_positions = [0.0, 0.0]  # 闭合
        
        # 状态变量
        self.current_state = GraspState.IDLE
        self.target_pose: Optional[Point] = None
        self.grasp_success = False
        
        # 数据收集（用于statistics_analyzer）
        self.trajectory_data = []
        self.grasp_attempts = []
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 初始化 arm_controller（如果可用且启用）
        self.arm_controller = None
        if self.use_arm_controller and ARM_CONTROLLER_AVAILABLE and ArmController is not None:
            try:
                self.arm_controller = ArmController(arm_name=self.arm_group, namespace=self.namespace)
                if not self.arm_controller.wait_for_server(timeout_sec=5.0):
                    self.get_logger().warn('arm_controller 服务器未就绪，将使用 MoveIt2 直接控制')
                    self.arm_controller = None
                else:
                    self.get_logger().info(f'使用 openarm_control/arm_controller.py 控制 {self.arm_group}')
            except Exception as e:
                self.get_logger().warn(f'初始化 arm_controller 失败: {str(e)}，将使用 MoveIt2')
                self.arm_controller = None
        
        # 初始化MoveIt2接口（基于 openarm_moveit_config 配置）
        if MoveIt2 is not None:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=self.joint_names,
                base_link_name=self.base_link,
                end_effector_name=self.end_effector_link,
                group_name=self.arm_group,
                callback_group=self.callback_group,
                use_move_group_action=True,
            )
            self.moveit2.planner_id = self.planner_id
            self.moveit2.max_velocity = self.max_velocity
            self.moveit2.max_acceleration = self.max_acceleration
            
            # 初始化夹爪接口
            if GripperInterface is not None:
                self.gripper = GripperInterface(
                    node=self,
                    gripper_joint_names=self.gripper_joint_names,
                    open_gripper_joint_positions=self.open_gripper_positions,
                    closed_gripper_joint_positions=self.closed_gripper_positions,
                    gripper_group_name=self.gripper_group,
                    callback_group=self.callback_group,
                )
            else:
                self.gripper = None
                self.get_logger().warn("GripperInterface not available")
        else:
            self.moveit2 = None
            self.gripper = None
            self.get_logger().error("MoveIt2 not available! Please install pymoveit2.")
        
        # 订阅物体位置话题
        self.target_pose_sub = self.create_subscription(
            Point,
            '/target_pose',
            self.target_pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 发布状态信息
        from std_msgs.msg import String
        self.state_pub = self.create_publisher(String, '/grasp_state', 10)
        
        # 创建定时器用于状态机执行
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.get_logger().info(f'抓取规划节点已启动 (规划组: {self.arm_group}, 规划器: {self.planner_id})')
        
        # 启动时打开夹爪
        if self.gripper is not None:
            self.gripper.open()
            self.gripper.wait_until_executed()
    
    def target_pose_callback(self, msg: Point):
        """接收物体位置的回调函数"""
        if self.current_state == GraspState.IDLE:
            self.target_pose = msg
            self.get_logger().info(
                f'收到物体位置: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}'
            )
            # 开始抓取流程
            self.current_state = GraspState.MOVING_TO_PRE_GRASP
            self.publish_state()
        else:
            self.get_logger().warn(f'当前状态 {self.current_state.value}，忽略新的物体位置')
    
    def state_machine_callback(self):
        """状态机主循环"""
        if self.moveit2 is None:
            return
        
        if self.current_state == GraspState.IDLE:
            return
        
        # 检查MoveIt2执行状态
        if self.moveit2.query_state().name == "EXECUTING":
            return  # 正在执行，等待完成
        
        # 根据当前状态执行相应动作
        if self.current_state == GraspState.MOVING_TO_PRE_GRASP:
            self.execute_pre_grasp()
        elif self.current_state == GraspState.MOVING_TO_GRASP:
            self.execute_grasp_approach()
        elif self.current_state == GraspState.GRASPING:
            self.execute_grasp()
        elif self.current_state == GraspState.LIFTING:
            self.execute_lift()
        elif self.current_state == GraspState.MOVING_TO_PLACE:
            self.execute_move_to_place()
        elif self.current_state == GraspState.PLACING:
            self.execute_place()
        elif self.current_state == GraspState.RETURNING:
            self.execute_return()
    
    def execute_pre_grasp(self):
        """执行预抓取：移动到物体上方"""
        if self.target_pose is None:
            self.current_state = GraspState.FAILED
            return
        
        # 计算预抓取位置（物体上方）
        pre_grasp_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + self.pre_grasp_offset_z
        ]
        
        # 预抓取姿态：末端执行器垂直向下
        # 使用欧拉角转换为四元数：roll=π, pitch=0, yaw=0 (垂直向下)
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # 180度绕Y轴旋转
        
        self.get_logger().info(f'移动到预抓取位置: {pre_grasp_position}')
        
        try:
            # 使用MoveIt2执行运动规划
            # 注意：arm_controller目前仅作为可选接口，实际执行仍通过MoveIt2
            # 未来可以扩展为：MoveIt2规划 -> arm_controller执行
            self.moveit2.move_to_pose(
                position=pre_grasp_position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            
            # 等待执行完成
            start_time = time.time()
            if self.moveit2.wait_until_executed(timeout_sec=30.0):
                duration = time.time() - start_time
                # 记录轨迹数据
                self.trajectory_data.append({
                    'state': 'pre_grasp',
                    'duration': duration,
                    'timestamp': time.time()
                })
                self.current_state = GraspState.MOVING_TO_GRASP
                self.get_logger().info('预抓取位置到达')
            else:
                self.current_state = GraspState.FAILED
                self.get_logger().error('移动到预抓取位置失败')
        except Exception as e:
            self.get_logger().error(f'预抓取规划失败: {str(e)}')
            self.current_state = GraspState.FAILED
        
        self.publish_state()
    
    def execute_grasp_approach(self):
        """执行抓取接近：从预抓取位置移动到抓取位置"""
        if self.target_pose is None:
            self.current_state = GraspState.FAILED
            return
        
        # 计算抓取位置
        grasp_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + self.grasp_offset_z
        ]
        
        # 保持垂直向下的姿态
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]
        
        self.get_logger().info(f'移动到抓取位置: {grasp_position}')
        
        try:
            # 使用笛卡尔路径规划，确保直线下降
            self.moveit2.move_to_pose(
                position=grasp_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,  # 使用笛卡尔路径
                cartesian_max_step=0.01,  # 小步长确保平滑
            )
            if self.moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = GraspState.GRASPING
                self.get_logger().info('抓取位置到达')
            else:
                self.current_state = GraspState.FAILED
                self.get_logger().error('移动到抓取位置失败')
        except Exception as e:
            self.get_logger().error(f'抓取接近规划失败: {str(e)}')
            self.current_state = GraspState.FAILED
        
        self.publish_state()
    
    def execute_grasp(self):
        """执行抓取：闭合夹爪"""
        self.get_logger().info('闭合夹爪进行抓取')
        
        if self.gripper is not None:
            try:
                self.gripper.close()
                if self.gripper.wait_until_executed(timeout_sec=5.0):
                    self.current_state = GraspState.LIFTING
                    self.grasp_success = True
                    # 记录抓取尝试
                    self.grasp_attempts.append({
                        'timestamp': time.time(),
                        'success': True,
                        'arm': self.arm_group
                    })
                    self.get_logger().info('夹爪闭合成功')
                else:
                    self.current_state = GraspState.FAILED
                    self.grasp_attempts.append({
                        'timestamp': time.time(),
                        'success': False,
                        'arm': self.arm_group
                    })
                    self.get_logger().error('夹爪闭合失败')
            except Exception as e:
                self.get_logger().error(f'夹爪控制失败: {str(e)}')
                self.current_state = GraspState.FAILED
        else:
            # 如果没有夹爪接口，直接进入提升状态
            self.get_logger().warn('夹爪接口不可用，跳过抓取步骤')
            self.current_state = GraspState.LIFTING
        
        self.publish_state()
    
    def execute_lift(self):
        """执行提升：将物体提升到安全高度"""
        if self.target_pose is None:
            self.current_state = GraspState.FAILED
            return
        
        # 计算提升位置（保持x, y不变，提升z）
        lift_position = [
            self.target_pose.x,
            self.target_pose.y,
            self.target_pose.z + self.lift_height
        ]
        
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]
        
        self.get_logger().info(f'提升物体到: {lift_position}')
        
        try:
            self.moveit2.move_to_pose(
                position=lift_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
                cartesian_max_step=0.01,
            )
            if self.moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = GraspState.MOVING_TO_PLACE
                self.get_logger().info('物体提升成功')
            else:
                self.current_state = GraspState.FAILED
                self.get_logger().error('提升物体失败')
        except Exception as e:
            self.get_logger().error(f'提升规划失败: {str(e)}')
            self.current_state = GraspState.FAILED
        
        self.publish_state()
    
    def execute_move_to_place(self):
        """执行移动到放置位置"""
        # 放置位置（保持高度）
        place_position = [
            self.place_position[0],
            self.place_position[1],
            self.place_position[2]
        ]
        
        # 放置姿态：垂直向下
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]
        
        self.get_logger().info(f'移动到放置位置: {place_position}')
        
        try:
            self.moveit2.move_to_pose(
                position=place_position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            if self.moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = GraspState.PLACING
                self.get_logger().info('到达放置位置')
            else:
                self.current_state = GraspState.FAILED
                self.get_logger().error('移动到放置位置失败')
        except Exception as e:
            self.get_logger().error(f'移动到放置位置规划失败: {str(e)}')
            self.current_state = GraspState.FAILED
        
        self.publish_state()
    
    def execute_place(self):
        """执行放置：打开夹爪"""
        self.get_logger().info('打开夹爪放置物体')
        
        if self.gripper is not None:
            try:
                self.gripper.open()
                if self.gripper.wait_until_executed(timeout_sec=5.0):
                    self.current_state = GraspState.RETURNING
                    self.get_logger().info('物体放置成功')
                else:
                    self.current_state = GraspState.FAILED
                    self.get_logger().error('夹爪打开失败')
            except Exception as e:
                self.get_logger().error(f'夹爪控制失败: {str(e)}')
                self.current_state = GraspState.FAILED
        else:
            self.get_logger().warn('夹爪接口不可用，跳过放置步骤')
            self.current_state = GraspState.RETURNING
        
        self.publish_state()
    
    def execute_return(self):
        """执行返回：移动到初始位置"""
        # 返回一个安全的初始位置（可以根据实际情况调整）
        home_joint_positions = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        
        self.get_logger().info('返回初始位置')
        
        try:
            self.moveit2.move_to_configuration(home_joint_positions)
            if self.moveit2.wait_until_executed(timeout_sec=30.0):
                self.current_state = GraspState.COMPLETED
                self.get_logger().info('抓取任务完成！')
                
                # 保存数据到文件（用于statistics_analyzer）
                self.save_trajectory_data()
                self.save_grasp_results()
                
                # 重置状态，准备下一次抓取
                self.current_state = GraspState.IDLE
                self.target_pose = None
                self.grasp_success = False
            else:
                self.current_state = GraspState.FAILED
                self.get_logger().error('返回初始位置失败')
        except Exception as e:
            self.get_logger().error(f'返回规划失败: {str(e)}')
            self.current_state = GraspState.FAILED
        
        self.publish_state()
    
    def publish_state(self):
        """发布当前状态"""
        from std_msgs.msg import String
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)
    
    def save_trajectory_data(self):
        """保存轨迹数据到JSON文件"""
        if not self.trajectory_data:
            return
        
        try:
            import json
            from pathlib import Path
            from datetime import datetime
            
            results_dir = Path.home() / "openarm_ws" / "results" / "motion_control"
            results_dir.mkdir(parents=True, exist_ok=True)
            
            filename = results_dir / f"trajectories_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(self.trajectory_data, f, indent=2)
            
            self.get_logger().info(f'轨迹数据已保存: {filename}')
            self.trajectory_data = []  # 清空数据
        except Exception as e:
            self.get_logger().warn(f'保存轨迹数据失败: {str(e)}')
    
    def save_grasp_results(self):
        """保存抓取结果到CSV文件"""
        if not self.grasp_attempts:
            return
        
        try:
            import csv
            from pathlib import Path
            from datetime import datetime
            
            results_dir = Path.home() / "openarm_ws" / "results" / "motion_control"
            results_dir.mkdir(parents=True, exist_ok=True)
            
            filename = results_dir / "grasp_results.csv"
            file_exists = filename.exists()
            
            with open(filename, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['timestamp', 'success', 'arm'])
                if not file_exists:
                    writer.writeheader()
                writer.writerows(self.grasp_attempts)
            
            self.get_logger().info(f'抓取结果已保存: {filename}')
            self.grasp_attempts = []  # 清空数据
        except Exception as e:
            self.get_logger().warn(f'保存抓取结果失败: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = GraspPlanner()
    
    # 使用多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    
    # 在后台线程中运行执行器
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # 等待初始化
    node.create_rate(1.0).sleep()
    
    try:
        # 保持运行
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

