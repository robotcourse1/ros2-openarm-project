#!/usr/bin/env python3
"""
前向运动学测试脚本 - 验证机械臂末端位置
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import numpy as np


class ForwardKinematicsTest(Node):
    def __init__(self):
        super().__init__('fk_test')
        
        self.sub = self.create_subscription(
            JointState, '/mujoco_joint_states', self.joint_callback, 10
        )
        
        self.get_logger().info('前向运动学测试节点已启动')
    
    def joint_callback(self, msg: JointState):
        """计算并显示末端执行器的实际位置"""
        # 提取左臂关节角度
        left_joints = msg.position[0:7]
        
        # 计算末端位置（简化前向运动学）
        # 基座位置: 世界坐标系中 (0.8, -0.45, 0.35), 旋转90度
        # 左臂基座: 相对机器人基座 (0.0, 0.031, 0.698), 旋转-90度X
        
        # 机器人基座在世界坐标系
        base_world_x = 0.8
        base_world_y = -0.45
        base_world_z = 0.35
        
        # 左臂在机器人基座坐标系中的偏移
        arm_offset_y = 0.031
        arm_offset_z = 0.698
        
        # 简化的前向运动学（只考虑主要的Z方向延伸）
        theta1, theta2, theta3, theta4, theta5, theta6, theta7 = left_joints
        
        # 各连杆长度（Z方向）
        L0 = 0.0625  # link0-1
        L1 = 0.06    # link1-2
        L2 = 0.06625 # link2-3
        L3 = 0.15375 # link3-4
        L4 = 0.0955  # link4-5
        L5 = 0.1205  # link5-6
        L6 = 0.0375  # link6-7 近似
        gripper = 0.11  # 夹爪长度
        
        # 粗略估计（假设机械臂在XZ平面内运动）
        # Joint1: 绕Z轴旋转
        # Joint2, Joint4: 主要影响俯仰
        
        # 水平延伸（大致）
        r = (L2 + L3) * math.cos(theta2) + (L4 + L5) * math.cos(theta2 + theta4)
        
        # 垂直高度
        z_arm = (L0 + L1 + L2 + L3 + L4 + L5) * math.sin(theta2 + theta4/2)
        
        # 在机器人基座坐标系中
        x_base = r * math.cos(theta1)
        y_base = r * math.sin(theta1) + arm_offset_y
        z_base = arm_offset_z + z_arm - gripper
        
        # 转换到世界坐标系（考虑90度旋转）
        cos_yaw = math.cos(1.5708)
        sin_yaw = math.sin(1.5708)
        
        x_world = base_world_x + (cos_yaw * x_base - sin_yaw * y_base)
        y_world = base_world_y + (sin_yaw * x_base + cos_yaw * y_base)
        z_world = base_world_z + z_base
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'关节角度(度): [{", ".join([f"{math.degrees(a):.1f}" for a in left_joints])}]')
        self.get_logger().info(f'末端世界坐标(估算): x={x_world:.3f}, y={y_world:.3f}, z={z_world:.3f}')
        self.get_logger().info(f'目标位置(苹果):     x=0.850,  y=0.050,  z=0.790')
        self.get_logger().info(f'位置误差:           Δx={abs(x_world-0.85):.3f}, Δy={abs(y_world-0.05):.3f}, Δz={abs(z_world-0.79):.3f}')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
