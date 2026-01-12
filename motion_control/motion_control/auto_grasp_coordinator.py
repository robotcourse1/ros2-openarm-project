#!/usr/bin/env python3
"""
自动抓取协调器 - 监听视觉检测结果并自动触发抓取
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time


class AutoGraspCoordinator(Node):
    def __init__(self):
        super().__init__('auto_grasp_coordinator')
        
        # 订阅视觉检测的目标位置
        self.sub_target = self.create_subscription(
            Point, '/target_pose', self.target_callback, 10
        )
        
        # 订阅抓取状态
        self.sub_grasp_status = self.create_subscription(
            String, '/grasp_status', self.grasp_status_callback, 10
        )
        
        # 发布目标位置给状态机（转发）
        self.pub_grasp_target = self.create_publisher(Point, '/grasp_target', 10)
        
        # 状态变量
        self.is_grasping = False
        self.last_target_time = 0
        self.target_cooldown = 5.0  # 5秒冷却时间，避免重复抓取
        self.auto_mode = True  # 自动模式开关
        
        self.get_logger().info('🤖 自动抓取协调器已启动!')
        self.get_logger().info('📡 等待视觉检测结果...')
    
    def target_callback(self, msg: Point):
        """接收视觉检测的目标位置"""
        current_time = time.time()
        
        # 检查是否在冷却时间内
        if current_time - self.last_target_time < self.target_cooldown:
            return
        
        # 检查是否正在抓取
        if self.is_grasping:
            self.get_logger().info('⏸️ 正在执行抓取任务，忽略新目标')
            return
        
        # 检查自动模式
        if not self.auto_mode:
            self.get_logger().info('⏸️ 自动模式已关闭，忽略目标')
            return
        
        # 检查目标位置是否合理
        if msg.x < 0.5 or msg.x > 1.2 or abs(msg.y) > 0.5 or msg.z < 0.5 or msg.z > 1.2:
            self.get_logger().warn(f'⚠️ 目标位置不合理: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})，忽略')
            return
        
        # 发送抓取命令
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🎯 检测到新目标: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')
        self.get_logger().info('🚀 启动自动抓取...')
        self.get_logger().info('=' * 60)
        
        self.pub_grasp_target.publish(msg)
        self.is_grasping = True
        self.last_target_time = current_time
    
    def grasp_status_callback(self, msg: String):
        """监听抓取状态"""
        status = msg.data
        
        if status == 'SUCCESS':
            self.get_logger().info('✅ 抓取成功！')
            self.is_grasping = False
        elif status == 'FAILED':
            self.get_logger().info('❌ 抓取失败！')
            self.is_grasping = False


def main(args=None):
    rclpy.init(args=args)
    node = AutoGraspCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
