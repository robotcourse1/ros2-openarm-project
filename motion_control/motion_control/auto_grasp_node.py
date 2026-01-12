#!/usr/bin/env python3
"""
自动抓取节点 - 持续监听物体检测，自动触发抓取
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time


class AutoGraspNode(Node):
    def __init__(self):
        super().__init__('auto_grasp_node')
        
        # 订阅物体检测结果
        self.sub_detected_object = self.create_subscription(
            Point, '/target_pose', self.object_detected_callback, 10
        )
        
        # 订阅抓取状态
        self.sub_grasp_status = self.create_subscription(
            String, '/grasp_status', self.grasp_status_callback, 10
        )
        
        # 发布抓取触发命令
        self.pub_grasp_trigger = self.create_publisher(Point, '/target_pose', 10)
        
        # 状态变量
        self.is_grasping = False
        self.last_grasp_time = 0
        self.grasp_cooldown = 15.0  # 抓取冷却时间（秒）
        
        # 物体检测缓冲
        self.detected_objects = []
        self.detection_threshold = 5  # 需要连续检测5次才触发抓取
        
        # 定时器：检查是否有稳定的检测结果
        self.timer = self.create_timer(0.5, self.check_and_trigger_grasp)
        
        self.get_logger().info('🤖 自动抓取节点已启动！')
        self.get_logger().info('📡 等待物体检测结果...')
    
    def object_detected_callback(self, msg: Point):
        """接收到物体检测结果"""
        # 如果正在抓取，忽略新的检测
        if self.is_grasping:
            return
        
        # 添加到检测缓冲
        self.detected_objects.append({
            'position': msg,
            'time': time.time()
        })
        
        # 只保留最近1秒内的检测
        current_time = time.time()
        self.detected_objects = [
            obj for obj in self.detected_objects 
            if current_time - obj['time'] < 1.0
        ]
    
    def grasp_status_callback(self, msg: String):
        """更新抓取状态"""
        if msg.data == 'SUCCESS':
            self.is_grasping = False
            self.last_grasp_time = time.time()
            self.detected_objects.clear()
            self.get_logger().info('✅ 抓取完成！进入冷却期...')
        elif msg.data == 'FAILED':
            self.is_grasping = False
            self.get_logger().warn('❌ 抓取失败！')
    
    def check_and_trigger_grasp(self):
        """检查是否满足抓取条件并触发"""
        # 检查冷却时间
        if time.time() - self.last_grasp_time < self.grasp_cooldown:
            return
        
        # 检查是否正在抓取
        if self.is_grasping:
            return
        
        # 检查是否有足够的稳定检测
        if len(self.detected_objects) < self.detection_threshold:
            return
        
        # 计算最近检测的平均位置
        avg_x = sum(obj['position'].x for obj in self.detected_objects) / len(self.detected_objects)
        avg_y = sum(obj['position'].y for obj in self.detected_objects) / len(self.detected_objects)
        avg_z = sum(obj['position'].z for obj in self.detected_objects) / len(self.detected_objects)
        
        # 检查位置稳定性（标准差）
        std_x = (sum((obj['position'].x - avg_x)**2 for obj in self.detected_objects) / len(self.detected_objects))**0.5
        std_y = (sum((obj['position'].y - avg_y)**2 for obj in self.detected_objects) / len(self.detected_objects))**0.5
        std_z = (sum((obj['position'].z - avg_z)**2 for obj in self.detected_objects) / len(self.detected_objects))**0.5
        
        # 如果位置不稳定，等待更多检测
        if std_x > 0.05 or std_y > 0.05 or std_z > 0.05:
            self.get_logger().info(f'🔍 物体位置不稳定 (std: {std_x:.3f}, {std_y:.3f}, {std_z:.3f})，继续观察...')
            return
        
        # 触发抓取
        target = Point()
        target.x = avg_x
        target.y = avg_y
        target.z = avg_z
        
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'🎯 检测到稳定物体！位置: ({avg_x:.3f}, {avg_y:.3f}, {avg_z:.3f})')
        self.get_logger().info(f'🤖 触发自动抓取...')
        self.get_logger().info('=' * 80)
        
        # 发布抓取命令
        self.pub_grasp_trigger.publish(target)
        self.is_grasping = True
        self.detected_objects.clear()


def main(args=None):
    rclpy.init(args=args)
    node = AutoGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
