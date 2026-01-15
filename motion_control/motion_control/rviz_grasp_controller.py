#!/usr/bin/env python3
"""
RViz 抓取演示控制节点
控制 OpenArm 机器人在 RViz 中执行抓取动作
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point
import math
import time
from enum import Enum


class GraspPhase(Enum):
    IDLE = 0
    MOVE_TO_READY = 1
    APPROACH = 2
    DESCEND = 3
    CLOSE_GRIPPER = 4
    LIFT = 5
    SUCCESS = 6


class RvizGraspController(Node):
    def __init__(self):
        super().__init__('rviz_grasp_controller')
        
        # 关节名称（与 URDF 中的关节名称完全一致）
        # 右臂关节
        self.right_arm_joint_names = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3', 
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6', 
            'openarm_right_joint7'
        ]
        # 左臂关节
        self.left_arm_joint_names = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        # 当前关节位置
        self.current_right_positions = [0.0] * 7
        self.current_left_positions = [0.0] * 7
        
        # 目标关节位置
        self.target_right_positions = [0.0] * 7
        self.target_left_positions = [0.0] * 7
        
        # 抓取状态
        self.phase = GraspPhase.IDLE
        self.phase_start_time = time.time()
        
        # 目标物体位置（苹果，相对于桌子）
        # 桌子在 (0.8, 0, 0)，苹果相对偏移 (0.15, -0.05, 0.79)
        self.target_object = Point()
        self.target_object.x = 0.95  # 0.8 + 0.15
        self.target_object.y = -0.05
        self.target_object.z = 0.79
        
        # 发布关节状态
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)
        
        # 订阅目标位置
        self.sub_target = self.create_subscription(
            Point, '/target_pose', self.target_callback, 10
        )
        
        # 订阅控制命令
        self.sub_command = self.create_subscription(
            String, '/grasp_command', self.command_callback, 10
        )
        
        # 发布状态
        self.pub_status = self.create_publisher(String, '/grasp_status', 10)
        
        # 定时器：发布关节状态 (50Hz)
        self.timer = self.create_timer(0.02, self.update_and_publish)
        
        # 运动平滑参数
        self.smoothing_factor = 0.02  # 越小越平滑
        
        # 预定义的抓取姿态
        self.define_grasp_poses()
        
        self.get_logger().info('🤖 RViz 抓取控制器已启动!')
        self.get_logger().info('📝 发送 "start" 到 /grasp_command 开始抓取演示')
        self.get_logger().info('📝 或发送目标位置到 /target_pose')
    
    def define_grasp_poses(self):
        """
        定义各阶段的关节姿态
        关节限制：
        - joint1: (-1.396, 3.49)  肩部旋转
        - joint2: (-0.175, 3.316) 大臂前倾
        - joint3: (-1.571, 1.571) 大臂旋转
        - joint4: (0.0, 2.443)    肘部弯曲
        - joint5: (-1.571, 1.571) 前臂旋转
        - joint6: (-0.785, 0.785) 腕部俯仰
        - joint7: (-1.571, 1.571) 腕部旋转
        """
        # 初始/待机姿态 - 手臂自然下垂
        self.pose_idle = [0.0, 0.3, 0.0, 0.3, 0.0, 0.0, 0.0]
        
        # 准备姿态 - 手臂抬起，指向目标方向
        # joint1=0.8 向前偏转指向苹果
        # joint2=1.5 大臂前倾
        # joint4=0.6 肘部略微弯曲
        self.pose_ready = [0.8, 1.5, 0.0, 0.6, 0.0, 0.0, 0.0]
        
        # 接近姿态 - 在目标正上方
        # joint2=2.0 大臂更前倾
        # joint4=1.0 肘部弯曲更多
        self.pose_approach = [0.8, 2.0, 0.0, 1.0, 0.0, 0.2, 0.0]
        
        # 下降姿态 - 靠近物体，准备抓取
        # joint2=2.5 大臂最大前倾
        # joint4=1.8 肘部大幅弯曲使末端下降
        # joint6=0.4 腕部下倾
        self.pose_descend = [0.8, 2.5, 0.0, 1.8, 0.0, 0.4, 0.0]
        
        # 抓取姿态 - 夹紧物体（与下降相同，只是夹爪闭合）
        self.pose_grasp = [0.8, 2.5, 0.0, 1.8, 0.0, 0.4, 0.0]
        
        # 提升姿态 - 抓住后提升
        self.pose_lift = [0.8, 1.8, 0.0, 1.0, 0.0, 0.2, 0.0]
        
        # 左臂保持不动
        self.left_arm_idle = [0.0, 0.3, 0.0, 0.3, 0.0, 0.0, 0.0]
    
    def target_callback(self, msg: Point):
        """接收目标位置"""
        self.target_object = msg
        self.get_logger().info(f'🎯 收到目标位置: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
        # 自动开始抓取
        if self.phase == GraspPhase.IDLE:
            self.start_grasp()
    
    def command_callback(self, msg: String):
        """接收控制命令"""
        cmd = msg.data.lower().strip()
        if cmd == 'start':
            self.start_grasp()
        elif cmd == 'stop' or cmd == 'reset':
            self.reset()
        else:
            self.get_logger().warn(f'未知命令: {cmd}')
    
    def start_grasp(self):
        """开始抓取流程"""
        self.get_logger().info('🚀 开始抓取流程!')
        self.phase = GraspPhase.MOVE_TO_READY
        self.phase_start_time = time.time()
        self.target_right_positions = list(self.pose_ready)
        self.target_left_positions = list(self.left_arm_idle)
        self.publish_status('STARTED')
    
    def reset(self):
        """重置到初始状态"""
        self.get_logger().info('🔄 重置到初始状态')
        self.phase = GraspPhase.IDLE
        self.target_right_positions = list(self.pose_idle)
        self.target_left_positions = list(self.left_arm_idle)
        self.publish_status('RESET')
    
    def update_and_publish(self):
        """更新关节位置并发布"""
        # 平滑更新右臂关节位置
        for i in range(7):
            diff = self.target_right_positions[i] - self.current_right_positions[i]
            self.current_right_positions[i] += diff * self.smoothing_factor
        
        # 平滑更新左臂关节位置
        for i in range(7):
            diff = self.target_left_positions[i] - self.current_left_positions[i]
            self.current_left_positions[i] += diff * self.smoothing_factor
        
        # 状态机更新
        self.update_state_machine()
        
        # 发布关节状态
        self.publish_joint_states()
    
    def update_state_machine(self):
        """状态机逻辑"""
        elapsed = time.time() - self.phase_start_time
        
        if self.phase == GraspPhase.IDLE:
            pass
        
        elif self.phase == GraspPhase.MOVE_TO_READY:
            if elapsed > 4.0:  # 4秒后进入下一阶段
                self.get_logger().info('📍 进入接近阶段')
                self.phase = GraspPhase.APPROACH
                self.phase_start_time = time.time()
                self.target_right_positions = list(self.pose_approach)
                self.publish_status('APPROACHING')
        
        elif self.phase == GraspPhase.APPROACH:
            if elapsed > 4.0:
                self.get_logger().info('⬇️ 进入下降阶段')
                self.phase = GraspPhase.DESCEND
                self.phase_start_time = time.time()
                self.target_right_positions = list(self.pose_descend)
                self.publish_status('DESCENDING')
        
        elif self.phase == GraspPhase.DESCEND:
            if elapsed > 4.0:
                self.get_logger().info('✋ 抓取物体')
                self.phase = GraspPhase.CLOSE_GRIPPER
                self.phase_start_time = time.time()
                self.target_right_positions = list(self.pose_grasp)
                self.publish_status('GRASPING')
        
        elif self.phase == GraspPhase.CLOSE_GRIPPER:
            if elapsed > 2.0:
                self.get_logger().info('⬆️ 提升物体')
                self.phase = GraspPhase.LIFT
                self.phase_start_time = time.time()
                self.target_right_positions = list(self.pose_lift)
                self.publish_status('LIFTING')
        
        elif self.phase == GraspPhase.LIFT:
            if elapsed > 4.0:
                self.get_logger().info('🎉 抓取成功!')
                self.phase = GraspPhase.SUCCESS
                self.phase_start_time = time.time()
                self.publish_status('SUCCESS')
        
        elif self.phase == GraspPhase.SUCCESS:
            if elapsed > 5.0:
                self.get_logger().info('🔄 准备下一次抓取')
                self.reset()
    
    def publish_joint_states(self):
        """发布关节状态消息"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # 所有关节名称（左臂 + 右臂）
        msg.name = self.left_arm_joint_names + self.right_arm_joint_names
        
        # 所有关节位置
        msg.position = list(self.current_left_positions) + list(self.current_right_positions)
        
        # 速度和力矩（可选）
        msg.velocity = []
        msg.effort = []
        
        self.pub_joint_state.publish(msg)
    
    def publish_status(self, status: str):
        """发布状态消息"""
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RvizGraspController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
