#!/usr/bin/env python3
"""
夹爪控制器节点
控制机械臂末端执行器的开合 - 直接控制MuJoCo关节
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # 订阅夹爪命令 (open/close)
        self.sub_gripper_cmd = self.create_subscription(
            String, '/gripper_command', self.gripper_command_callback, 10
        )
        
        # 发布夹爪状态
        self.pub_gripper_state = self.create_publisher(String, '/gripper_state', 10)
        
        # NOTE: Do NOT publish gripper joints to /joint_command.
        # The MuJoCo bridge already consumes /gripper_command directly and applies actuators.
        # Publishing here conflicts with arm JointState commands and can cause twitching.
        
        # 夹爪状态
        self.gripper_open = True
        self.gripper_position = 0.04  # 0.04 = 完全打开, 0.0 = 完全闭合（关节范围）
        
        # 定时发布状态
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info('夹爪控制器已启动!')
    
    def gripper_command_callback(self, msg: String):
        """处理夹爪控制命令"""
        command = msg.data.lower()
        
        if command == 'open':
            self.gripper_open = True
            self.gripper_position = 0.04  # 完全打开
            self.get_logger().info('夹爪打开')
        elif command == 'close':
            self.gripper_open = False
            self.gripper_position = 0.0  # 完全闭合
            self.get_logger().info('夹爪闭合')
        else:
            self.get_logger().warn(f'未知命令: {command}')
    
    def publish_state(self):
        """发布当前夹爪状态"""
        msg = String()
        msg.data = 'open' if self.gripper_open else 'closed'
        self.pub_gripper_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
