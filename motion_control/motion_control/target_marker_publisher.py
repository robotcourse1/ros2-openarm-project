#!/usr/bin/env python3
"""
目标标记发布器 - 在 RViz 中显示目标位置和抓取轨迹
订阅 /target_pose 并发布可视化标记
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA


class TargetMarkerPublisher(Node):
    def __init__(self):
        super().__init__('target_marker_publisher')
        
        # 订阅目标位置
        self.sub_target = self.create_subscription(
            Point, '/target_pose', self.target_callback, 10
        )
        
        # 订阅末端执行器位置（用于绘制轨迹）
        self.sub_ee_pose = self.create_subscription(
            PoseStamped, '/right_ee_pose_world', self.ee_pose_callback, 10
        )
        
        # 发布标记
        self.pub_marker = self.create_publisher(Marker, '/target_marker', 10)
        self.pub_marker_array = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self.pub_path = self.create_publisher(Path, '/grasp_path', 10)
        
        # 存储轨迹点
        self.path = Path()
        self.path.header.frame_id = 'world'
        self.max_path_points = 500  # 最多保存500个点
        
        # 当前目标
        self.current_target = None
        
        # 定时发布标记（10Hz）
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('🎯 目标标记发布器已启动')
    
    def target_callback(self, msg: Point):
        """接收目标位置"""
        self.current_target = msg
        self.get_logger().info(f'🎯 收到新目标: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
        
        # 清空轨迹，开始新的抓取
        self.path.poses.clear()
    
    def ee_pose_callback(self, msg: PoseStamped):
        """记录末端执行器轨迹"""
        # 添加到轨迹
        self.path.poses.append(msg)
        
        # 限制轨迹长度
        if len(self.path.poses) > self.max_path_points:
            self.path.poses.pop(0)
    
    def publish_markers(self):
        """发布可视化标记"""
        now = self.get_clock().now().to_msg()
        
        # 1. 发布目标位置标记（红色球体）
        if self.current_target:
            target_marker = Marker()
            target_marker.header.stamp = now
            target_marker.header.frame_id = 'world'
            target_marker.ns = 'target'
            target_marker.id = 0
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            target_marker.pose.position.x = self.current_target.x
            target_marker.pose.position.y = self.current_target.y
            target_marker.pose.position.z = self.current_target.z
            target_marker.pose.orientation.w = 1.0
            
            # 球体大小（苹果大小约5cm）
            target_marker.scale.x = 0.06
            target_marker.scale.y = 0.06
            target_marker.scale.z = 0.06
            
            # 红色
            target_marker.color.r = 1.0
            target_marker.color.g = 0.2
            target_marker.color.b = 0.2
            target_marker.color.a = 0.9
            
            target_marker.lifetime.sec = 0  # 永久显示
            
            self.pub_marker.publish(target_marker)
            
            # 2. 发布目标上方的文字标记
            text_marker = Marker()
            text_marker.header.stamp = now
            text_marker.header.frame_id = 'world'
            text_marker.ns = 'target_text'
            text_marker.id = 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = self.current_target.x
            text_marker.pose.position.y = self.current_target.y
            text_marker.pose.position.z = self.current_target.z + 0.15  # 在目标上方
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.05  # 文字高度
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f'Target\n({self.current_target.x:.2f}, {self.current_target.y:.2f}, {self.current_target.z:.2f})'
            
            # 发布标记数组
            marker_array = MarkerArray()
            marker_array.markers.append(target_marker)
            marker_array.markers.append(text_marker)
            
            # 3. 添加抓取区域指示（圆柱体，表示接近方向）
            approach_marker = Marker()
            approach_marker.header.stamp = now
            approach_marker.header.frame_id = 'world'
            approach_marker.ns = 'approach_zone'
            approach_marker.id = 2
            approach_marker.type = Marker.CYLINDER
            approach_marker.action = Marker.ADD
            
            approach_marker.pose.position.x = self.current_target.x
            approach_marker.pose.position.y = self.current_target.y
            approach_marker.pose.position.z = self.current_target.z + 0.1  # 目标上方
            approach_marker.pose.orientation.w = 1.0
            
            approach_marker.scale.x = 0.08  # 直径
            approach_marker.scale.y = 0.08
            approach_marker.scale.z = 0.2   # 高度
            
            # 半透明绿色
            approach_marker.color.r = 0.2
            approach_marker.color.g = 1.0
            approach_marker.color.b = 0.2
            approach_marker.color.a = 0.3
            
            marker_array.markers.append(approach_marker)
            
            self.pub_marker_array.publish(marker_array)
        
        # 4. 发布末端轨迹
        if len(self.path.poses) > 1:
            self.path.header.stamp = now
            self.pub_path.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TargetMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
