#!/usr/bin/env python3
"""
实时末端位置测试 - 计算右臂末端实际位置
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class EndEffectorPositionTest(Node):
    def __init__(self):
        super().__init__('ee_position_test')
        
        self.sub_joint_state = self.create_subscription(
            JointState, '/mujoco_joint_states', self.joint_state_callback, 10
        )
        
        self.get_logger().info('🔍 末端位置测试节点已启动! 正在监听关节状态...')
    
    def joint_state_callback(self, msg: JointState):
        """计算并显示末端位置"""
        # 找到右臂关节
        right_joints = []
        for i, name in enumerate(msg.name):
            if 'right_joint' in name and i < len(msg.position):
                right_joints.append(msg.position[i])
        
        if len(right_joints) < 7:
            return
        
        # 右臂关节角度
        q1, q2, q3, q4, q5, q6, q7 = right_joints
        
        # 计算末端位置（简化的DH变换）
        # 机器人基座：(0.8, -0.45, 0.35)，旋转90度
        # 右臂基座偏移：(0, -0.031, 0.698) 在机器人坐标系
        
        robot_base_x = 0.8
        robot_base_y = -0.45
        robot_base_z = 0.35
        robot_yaw = math.pi / 2  # 90度
        
        # 右臂基座在世界坐标系
        arm_base_x = robot_base_x + math.sin(robot_yaw) * 0.031
        arm_base_y = robot_base_y - math.cos(robot_yaw) * 0.031
        arm_base_z = robot_base_z + 0.698
        
        # 简化的末端位置估算（只考虑主要的臂展）
        # L1-L5是各段长度
        L0 = 0.0625  # joint1 高度
        L1 = 0.06625  # joint2-3
        L2 = 0.15375  # joint3-4
        L3 = 0.0955   # joint4-5
        L4 = 0.1205   # joint5-6
        L5 = 0.08     # 夹爪
        
        # 在右臂局部坐标系中计算（简化，只考虑主要关节）
        # Joint2向前倾斜，Joint4肘部弯曲
        forward = math.cos(q2) * L1 + math.cos(q2 + q4) * (L2 + L3 + L4 + L5)
        down = -math.sin(q2) * L1 - math.sin(q2 + q4) * (L2 + L3 + L4 + L5)
        
        # 转换到世界坐标系（考虑Joint1水平旋转）
        x_offset = forward * math.cos(q1)
        y_offset = forward * math.sin(q1)
        z_offset = down + L0
        
        # 最终世界坐标
        ee_x = arm_base_x + x_offset
        ee_y = arm_base_y + y_offset
        ee_z = arm_base_z + z_offset
        
        # 目标位置（苹果）
        target_x = 0.85
        target_y = -0.15
        target_z = 0.79
        
        # 计算误差
        error_x = target_x - ee_x
        error_y = target_y - ee_y
        error_z = target_z - ee_z
        error_3d = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'右臂关节角度(度): [{", ".join([f"{math.degrees(q):.1f}" for q in right_joints])}]')
        self.get_logger().info(f'右臂基座位置: ({arm_base_x:.3f}, {arm_base_y:.3f}, {arm_base_z:.3f})')
        self.get_logger().info(f'末端估算位置: ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f})')
        self.get_logger().info(f'苹果目标位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
        self.get_logger().info(f'位置误差: Δx={error_x:.3f}, Δy={error_y:.3f}, Δz={error_z:.3f}, 3D误差={error_3d:.3f}m')
        
        if error_z > 0.05:
            self.get_logger().warn(f'⚠️ 末端在目标上方 {error_z*100:.1f}cm！需要增加下降距离')
        elif error_z < -0.05:
            self.get_logger().warn(f'⚠️ 末端在目标下方 {abs(error_z)*100:.1f}cm！需要减少下降距离')
        
        if error_3d > 0.1:
            self.get_logger().warn(f'⚠️ 末端距离目标过远：{error_3d*100:.1f}cm')
        elif error_3d < 0.03:
            self.get_logger().info(f'✅ 末端接近目标：{error_3d*100:.1f}cm')
        
        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPositionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
