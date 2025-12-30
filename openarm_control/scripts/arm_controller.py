#!/usr/bin/env python3
"""
OpenArm Arm Controller
基于MoveIt2的轨迹执行控制器
支持单臂和双臂控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from typing import List, Optional
import time


class ArmController(Node):
    """基于MoveIt2的机械臂控制器"""
    
    def __init__(self, arm_name: str = 'left_arm', namespace: str = ''):
        """
        初始化机械臂控制器
        
        Args:
            arm_name: 机械臂名称 ('left_arm' 或 'right_arm')
            namespace: ROS命名空间（用于双臂配置）
        """
        node_name = f'{arm_name}_controller' if not namespace else f'{namespace}/{arm_name}_controller'
        super().__init__(node_name)
        
        self.arm_name = arm_name
        self.namespace = namespace
        
        # 根据arm_name确定关节名称
        if 'left' in arm_name:
            self.joint_names = [
                'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
                'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
                'openarm_left_joint7'
            ]
            self.controller_name = 'left_arm_controller'
        else:
            self.joint_names = [
                'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
                'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
                'openarm_right_joint7'
            ]
            self.controller_name = 'right_arm_controller'
        
        # 创建Action客户端
        action_topic = f'{self.controller_name}/follow_joint_trajectory'
        if namespace:
            action_topic = f'{namespace}/{action_topic}'
        
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_topic,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info(f'{arm_name}控制器已初始化，Action话题: {action_topic}')
    
    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """
        等待Action服务器就绪
        
        Args:
            timeout_sec: 超时时间（秒）
        
        Returns:
            True if server is ready, False otherwise
        """
        self.get_logger().info(f'等待{self.controller_name} Action服务器...')
        if self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().info(f'{self.controller_name} Action服务器已就绪')
            return True
        else:
            self.get_logger().error(f'{self.controller_name} Action服务器未就绪（超时{timeout_sec}秒）')
            return False
    
    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        timeout_sec: float = 30.0
    ) -> bool:
        """
        执行关节轨迹
        
        Args:
            trajectory: 关节轨迹消息
            timeout_sec: 执行超时时间（秒）
        
        Returns:
            True if execution succeeded, False otherwise
        """
        if not self.action_client.server_is_ready():
            self.get_logger().error('Action服务器未就绪')
            return False
        
        # 确保轨迹的关节名称正确
        trajectory.joint_names = self.joint_names
        
        # 创建目标
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # 发送目标
        self.get_logger().info(f'发送轨迹到{self.controller_name}，包含{len(trajectory.points)}个点')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        
        # 等待发送完成
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        if not send_goal_future.done():
            self.get_logger().error('发送目标超时')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        self.get_logger().info('目标已接受，等待执行完成...')
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                result = result_future.result().result
                if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    self.get_logger().info('轨迹执行成功')
                    return True
                else:
                    self.get_logger().error(f'轨迹执行失败，错误代码: {result.error_code}')
                    return False
            
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f'轨迹执行超时（{timeout_sec}秒）')
                return False
        
        return False
    
    def get_current_joint_positions(self) -> Optional[List[float]]:
        """
        获取当前关节位置（从joint_states话题）
        
        Returns:
            当前关节位置列表，如果获取失败返回None
        """
        try:
            from sensor_msgs.msg import JointState
            from rclpy.qos import qos_profile_sensor_data
            
            # 创建临时订阅者获取当前状态
            msg = None
            def callback(joint_state_msg):
                nonlocal msg
                msg = joint_state_msg
            
            sub = self.create_subscription(
                JointState,
                '/joint_states',
                callback,
                qos_profile_sensor_data
            )
            
            # 等待消息
            rclpy.spin_once(self, timeout_sec=1.0)
            self.destroy_subscription(sub)
            
            if msg is not None:
                positions = []
                for joint_name in self.joint_names:
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        positions.append(msg.position[idx])
                    else:
                        self.get_logger().warn(f'关节 {joint_name} 不在joint_states中')
                        return None
                return positions
        except Exception as e:
            self.get_logger().warn(f'获取当前关节位置失败: {str(e)}')
        
        return None
    
    def move_to_joint_positions(
        self,
        joint_positions: List[float],
        duration: float = 5.0,
        timeout_sec: float = 30.0
    ) -> bool:
        """
        移动到指定关节位置
        
        Args:
            joint_positions: 目标关节位置列表（7个关节）
            duration: 运动持续时间（秒）
            timeout_sec: 执行超时时间（秒）
        
        Returns:
            True if movement succeeded, False otherwise
        """
        if len(joint_positions) != len(self.joint_names):
            self.get_logger().error(
                f'关节位置数量不匹配：期望{len(self.joint_names)}，实际{len(joint_positions)}'
            )
            return False
        
        # 创建轨迹
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # 获取当前关节位置
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            self.get_logger().warn('无法获取当前关节位置，使用默认值0')
            current_positions = [0.0] * len(self.joint_names)
        
        # 起始点（当前状态）
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.velocities = [0.0] * len(self.joint_names)
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        trajectory.points.append(start_point)
        
        # 目标点
        end_point = JointTrajectoryPoint()
        end_point.positions = joint_positions
        end_point.velocities = [0.0] * len(self.joint_names)
        end_point.time_from_start.sec = int(duration)
        end_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points.append(end_point)
        
        return self.execute_trajectory(trajectory, timeout_sec)
    
    def stop(self):
        """停止当前运动"""
        # 取消所有目标
        if self.action_client.server_is_ready():
            # 这里可以添加取消逻辑
            self.get_logger().info('停止运动')


def main(args=None):
    """主函数：测试控制器"""
    rclpy.init(args=args)
    
    # 创建左臂控制器
    left_controller = ArmController(arm_name='left_arm')
    
    # 等待服务器
    if not left_controller.wait_for_server():
        left_controller.get_logger().error('无法连接到Action服务器')
        left_controller.destroy_node()
        rclpy.shutdown()
        return
    
    # 测试：移动到初始位置
    home_positions = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
    left_controller.get_logger().info('测试：移动到初始位置')
    success = left_controller.move_to_joint_positions(home_positions, duration=5.0)
    
    if success:
        left_controller.get_logger().info('测试成功')
    else:
        left_controller.get_logger().error('测试失败')
    
    left_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

