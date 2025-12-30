#!/usr/bin/env python3
"""
测试脚本：发布测试物体位置以触发抓取流程
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time


class TestGraspPublisher(Node):
    """测试抓取的发布节点"""
    
    def __init__(self):
        super().__init__('test_grasp_publisher')
        self.publisher = self.create_publisher(Point, '/target_pose', 10)
        self.get_logger().info('测试抓取发布节点已启动')
    
    def publish_target(self, x, y, z):
        """发布物体位置"""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.publisher.publish(msg)
        self.get_logger().info(f'发布物体位置: x={x:.3f}, y={y:.3f}, z={z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = TestGraspPublisher()
    
    # 等待一下确保订阅者已连接
    time.sleep(2.0)
    
    # 发布测试物体位置
    # 注意：位置应该在机器人工作空间内
    test_positions = [
        (0.5, 0.0, 0.3),   # 前方中心
        (0.4, 0.2, 0.3),   # 右前方
        (0.4, -0.2, 0.3),  # 左前方
    ]
    
    for i, (x, y, z) in enumerate(test_positions):
        node.get_logger().info(f'测试 {i+1}/{len(test_positions)}')
        node.publish_target(x, y, z)
        
        # 等待抓取完成（可以根据实际情况调整）
        if i < len(test_positions) - 1:
            node.get_logger().info('等待30秒后发布下一个位置...')
            time.sleep(30.0)
    
    node.get_logger().info('所有测试位置已发布')
    
    # 保持节点运行以便观察
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

