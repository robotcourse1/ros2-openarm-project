#!/usr/bin/env python3
"""
简单的夹爪控制节点（备用方案）
如果pymoveit2的GripperInterface不可用，可以使用此节点
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GripperController(Node):
    """简单的夹爪控制器"""
    
    def __init__(self):
        super().__init__('gripper_controller')
        
        # 声明参数
        self.declare_parameter('gripper_joint_names', ['openarm_left_finger_joint1'])
        self.declare_parameter('action_name', '/left_hand_controller/follow_joint_trajectory')
        self.declare_parameter('open_position', 0.03)
        self.declare_parameter('closed_position', 0.0)
        
        # 获取参数
        self.gripper_joint_names = self.get_parameter('gripper_joint_names').get_parameter_value().string_array_value
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.open_position = self.get_parameter('open_position').get_parameter_value().double_value
        self.closed_position = self.get_parameter('closed_position').get_parameter_value().double_value
        
        # 创建动作客户端
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name
        )
        
        self.get_logger().info(f'夹爪控制器初始化完成 (动作: {self.action_name})')
    
    def wait_for_server(self, timeout_sec=5.0):
        """等待动作服务器上线"""
        if self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().info('夹爪动作服务器已上线')
            return True
        else:
            self.get_logger().warn('夹爪动作服务器未上线')
            return False
    
    def move_gripper(self, position: float, duration_sec: float = 1.0):
        """移动夹爪到指定位置"""
        if not self.action_client.server_is_ready():
            self.get_logger().error('夹爪动作服务器未就绪')
            return False
        
        # 创建目标消息
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = [position] * len(self.gripper_joint_names)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        
        # 发送目标
        self.get_logger().info(f'发送夹爪目标位置: {position}')
        future = self.action_client.send_goal_async(goal_msg)
        
        # 等待结果
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('夹爪目标被拒绝')
            return False
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        if result.error_code == 0:
            self.get_logger().info('夹爪移动完成')
            return True
        else:
            self.get_logger().error(f'夹爪移动失败，错误码: {result.error_code}')
            return False
    
    def open(self):
        """打开夹爪"""
        return self.move_gripper(self.open_position)
    
    def close(self):
        """闭合夹爪"""
        return self.move_gripper(self.closed_position)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    
    # 等待服务器
    node.wait_for_server()
    
    # 测试：打开和闭合
    node.get_logger().info('测试：打开夹爪')
    node.open()
    
    node.create_rate(2.0).sleep()
    
    node.get_logger().info('测试：闭合夹爪')
    node.close()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

