#!/usr/bin/env python3
"""
运动规划节点 - 使用简单逆运动学求解器
订阅目标位置，发布关节控制命令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import math


class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        
        # 订阅目标位置（来自视觉感知）
        self.sub_target = self.create_subscription(
            Point, '/target_pose', self.target_callback, 10
        )
        
        # 发布关节命令
        self.pub_joint_cmd = self.create_publisher(JointState, '/joint_command', 10)
        
        # 订阅当前关节状态
        self.sub_joint_state = self.create_subscription(
            JointState, '/mujoco_joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_state = None
        
        # DH参数（简化版本，根据OpenArm实际结构）
        # 这里使用基于你的URDF的简化链条参数
        self.link_lengths = [0.0625, 0.06, 0.06625, 0.15375, 0.0955, 0.1205, 0.10]
        
        self.get_logger().info('运动规划节点已启动!')
    
    def joint_state_callback(self, msg):
        """保存当前关节状态"""
        self.current_joint_state = msg
    
    def target_callback(self, msg: Point):
        """接收目标位置并计算逆运动学"""
        self.get_logger().info(f'收到目标位置: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')
        
        # 使用左臂进行抓取（也可以根据位置选择左/右臂）
        joint_angles = self.simple_ik_left_arm(msg.x, msg.y, msg.z)
        
        if joint_angles is not None:
            self.publish_joint_command(joint_angles, arm='left')
            self.get_logger().info(f'逆运动学求解成功，发送关节命令')
        else:
            self.get_logger().warn('逆运动学求解失败，目标可能超出工作空间')
    
    def simple_ik_left_arm(self, x, y, z):
        """
        简化的逆运动学求解（左臂）
        使用几何方法求解7自由度机械臂的逆解
        """
        # 基座位置偏移（从MuJoCo XML）
        base_x, base_y, base_z = 0.8, -0.45, 0.35
        left_offset = 0.031
        left_base_z = 0.698
        
        # 转换到机械臂基座坐标系
        px = x - base_x
        py = y - (base_y + left_offset)
        pz = z - (base_z + left_base_z)
        
        # 1. Joint 1 (底座旋转) - 指向目标
        theta1 = math.atan2(py, px)
        
        # 2. 计算到目标的水平距离
        r = math.sqrt(px**2 + py**2)
        
        # 3. 末端执行器位置调整（减去工具长度）
        tool_length = 0.10
        pz_adjusted = pz - tool_length
        
        # 4. Joint 2 和 Joint 4 的2R平面求解
        # 简化为两段臂长的平面问题
        L1 = 0.15  # 近似前臂长度
        L2 = 0.20  # 近似后臂长度
        
        d = math.sqrt(r**2 + pz_adjusted**2)
        
        if d > (L1 + L2) * 0.95:  # 检查是否在工作空间内（留5%余量）
            self.get_logger().warn(f'目标距离 {d:.3f}m 超出工作空间')
            return None
        
        # 余弦定理求解肘关节
        cos_theta4 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta4 = np.clip(cos_theta4, -1.0, 1.0)
        theta4 = math.acos(cos_theta4)
        
        # 求解肩关节
        alpha = math.atan2(pz_adjusted, r)
        beta = math.atan2(L2 * math.sin(theta4), L1 + L2 * math.cos(theta4))
        theta2 = alpha - beta
        
        # 其他关节使用默认/中间位置
        theta3 = 0.0  # 中间旋转
        theta5 = 0.0  # 腕部旋转
        theta6 = 0.0  # 腕部俯仰
        theta7 = 0.0  # 腕部滚转
        
        return [theta1, theta2, theta3, theta4, theta5, theta6, theta7]
    
    def publish_joint_command(self, joint_angles, arm='left'):
        """发布关节控制命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 生成关节名称
        if arm == 'left':
            msg.name = [f'openarm_left_joint{i}' for i in range(1, 8)]
        else:
            msg.name = [f'openarm_right_joint{i}' for i in range(1, 8)]
        
        msg.position = joint_angles
        
        self.pub_joint_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
