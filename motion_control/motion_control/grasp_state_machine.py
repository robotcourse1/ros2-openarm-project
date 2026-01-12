#!/usr/bin/env python3
"""
抓取状态机 - 完整的抓取流程控制
状态流程: IDLE → MOVE_TO_HIGH → DESCEND → CLOSE_GRIPPER → LIFT → SUCCESS → IDLE

策略：让机械臂先移动到物体正上方高处，然后垂直向下抓取
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
import math
from enum import Enum


class GraspState(Enum):
    """抓取状态枚举"""
    IDLE = 0
    MOVE_TO_HIGH = 1      # 移动到高位（物体正上方）
    DESCEND = 2           # 垂直下降
    CLOSE_GRIPPER = 3     # 闭合夹爪
    LIFT = 4              # 提升
    SUCCESS = 5
    FAILED = 6


class GraspStateMachine(Node):
    def __init__(self):
        super().__init__('grasp_state_machine')
        
        # 订阅目标位置（来自视觉）
        self.sub_target = self.create_subscription(
            Point, '/target_pose', self.target_callback, 10
        )
        
        # 订阅夹爪状态
        self.sub_gripper_state = self.create_subscription(
            String, '/gripper_state', self.gripper_state_callback, 10
        )
        
        # 订阅关节状态
        self.sub_joint_state = self.create_subscription(
            JointState, '/mujoco_joint_states', self.joint_state_callback, 10
        )
        
        # 订阅右臂末端位置（用于调试）
        self.sub_ee_pose = self.create_subscription(
            PoseStamped, '/right_ee_pose_world', self.ee_pose_callback, 10
        )
        
        # 发布控制指令
        self.pub_joint_cmd = self.create_publisher(JointState, '/joint_command', 10)
        self.pub_gripper_cmd = self.create_publisher(String, '/gripper_command', 10)
        self.pub_grasp_status = self.create_publisher(String, '/grasp_status', 10)
        
        # 状态变量
        self.current_state = GraspState.IDLE
        self.target_position = None
        self.gripper_state = 'open'
        self.current_joint_positions = None
        self.current_ee_position = None  # 末端执行器位置
        self.state_start_time = time.time()
        
        # 状态机定时器（10Hz）
        self.timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info('🤖 抓取状态机已启动! 等待目标...')
    
    def target_callback(self, msg: Point):
        """接收新的目标位置"""
        if self.current_state == GraspState.IDLE:
            self.target_position = msg
            self.current_state = GraspState.MOVE_TO_HIGH
            self.state_start_time = time.time()
            self.get_logger().info(f'🎯 新目标: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) - 开始抓取')
    
    def gripper_state_callback(self, msg: String):
        """更新夹爪状态"""
        self.gripper_state = msg.data
    
    def joint_state_callback(self, msg: JointState):
        """更新当前关节状态"""
        self.current_joint_positions = dict(zip(msg.name, msg.position))
    
    def ee_pose_callback(self, msg: PoseStamped):
        """更新末端执行器位置"""
        self.current_ee_position = msg.pose.position
    
    def state_machine_update(self):
        """状态机主循环"""
        elapsed = time.time() - self.state_start_time
        
        # 打印末端位置用于调试
        if self.current_ee_position and self.target_position and int(elapsed * 10) % 10 == 0:
            ee = self.current_ee_position
            tgt = self.target_position
            dist = math.sqrt((ee.x - tgt.x)**2 + (ee.y - tgt.y)**2 + (ee.z - tgt.z)**2)
            self.get_logger().info(f'📍 末端位置: ({ee.x:.3f}, {ee.y:.3f}, {ee.z:.3f}) | 目标: ({tgt.x:.3f}, {tgt.y:.3f}, {tgt.z:.3f}) | 距离: {dist:.3f}m')
        
        if self.current_state == GraspState.IDLE:
            pass
        
        elif self.current_state == GraspState.MOVE_TO_HIGH:
            # 第一阶段：移动到物体正上方高处
            if elapsed < 0.5:
                self.get_logger().info('📍 状态: MOVE_TO_HIGH - 移动到物体上方高位')
                self.send_gripper_command('open')
            
            # 发送高位姿态命令
            self.send_high_position_command()
            
            if elapsed > 4.0:
                self.get_logger().info('✅ MOVE_TO_HIGH 完成，进入 DESCEND')
                self.current_state = GraspState.DESCEND
                self.state_start_time = time.time()
        
        elif self.current_state == GraspState.DESCEND:
            # 第二阶段：垂直下降到抓取位置
            if elapsed < 0.5:
                self.get_logger().info('⬇️ 状态: DESCEND - 垂直下降抓取')
            
            # 发送下降姿态命令
            self.send_descend_command()
            
            if elapsed > 3.0:
                self.get_logger().info('✅ DESCEND 完成，进入 CLOSE_GRIPPER')
                self.current_state = GraspState.CLOSE_GRIPPER
                self.state_start_time = time.time()
        
        elif self.current_state == GraspState.CLOSE_GRIPPER:
            # 第三阶段：闭合夹爪
            if elapsed < 0.5:
                self.get_logger().info('✋ 状态: CLOSE_GRIPPER - 闭合夹爪')
            
            self.send_gripper_command('close')
            
            if elapsed > 1.5:
                self.get_logger().info('✅ CLOSE_GRIPPER 完成，进入 LIFT')
                self.current_state = GraspState.LIFT
                self.state_start_time = time.time()
        
        elif self.current_state == GraspState.LIFT:
            # 第四阶段：提升物体（回到高位）
            if elapsed < 0.5:
                self.get_logger().info('⬆️ 状态: LIFT - 提升物体')
            
            # 回到高位姿态
            self.send_high_position_command()
            
            if elapsed > 4.0:
                self.get_logger().info('✅ LIFT 完成，抓取成功！')
                self.current_state = GraspState.SUCCESS
                self.state_start_time = time.time()
        
        elif self.current_state == GraspState.SUCCESS:
            if elapsed < 0.5:
                self.get_logger().info('🎉 抓取成功！物体已抓住')
            
            self.publish_grasp_status('SUCCESS')
            
            if elapsed > 10.0:
                self.get_logger().info('🔄 准备接收下一个目标')
                self.current_state = GraspState.IDLE
                self.target_position = None
    
    def send_high_position_command(self):
        """
        发送高位姿态命令 - 让手臂悬停在物体正上方
        """
        if not self.target_position:
            return
        
        # 计算目标方向角（Joint1）
        robot_base_x = 0.8
        robot_base_y = -0.45
        robot_yaw = 1.5708
        
        arm_offset_y = -0.031
        arm_world_x = robot_base_x - math.sin(robot_yaw) * arm_offset_y
        arm_world_y = robot_base_y + math.cos(robot_yaw) * arm_offset_y
        
        dx = self.target_position.x - arm_world_x
        dy = self.target_position.y - arm_world_y
        
        # 计算水平距离
        horizontal_dist = math.sqrt(dx**2 + dy**2)
        
        angle_to_target = math.atan2(dy, dx)
        theta1 = angle_to_target - robot_yaw
        
        while theta1 < -1.396263:
            theta1 += 2 * math.pi
        while theta1 > 3.490659:
            theta1 -= 2 * math.pi
        
        # 根据水平距离调整关节角度
        # 距离越远，需要更大的前伸角度
        # 苹果距离约 0.35m，需要适当的前伸
        
        # joint2: 控制大臂前倾，值越大越往前伸
        # joint4: 控制肘部弯曲，值越大肘部弯曲越多，末端越低
        
        # 高位姿态：保持较高位置，准备下降
        joint_angles = [
            theta1,      # joint1: 指向目标
            2.2,         # joint2: 大臂前倾
            0.0,         # joint3: 不旋转
            0.8,         # joint4: 肘部适度弯曲
            0.0,         # joint5: 腕部不旋转
            0.0,         # joint6: 腕部水平
            0.0          # joint7: 末端不旋转
        ]
        
        self._publish_joint_command(joint_angles, 'right')
        self.get_logger().info(f'📐 高位姿态: theta1={math.degrees(theta1):.1f}° 水平距离={horizontal_dist:.3f}m')
    
    def send_descend_command(self):
        """
        发送下降姿态命令 - 垂直下降到抓取位置
        """
        if not self.target_position:
            return
        
        robot_base_x = 0.8
        robot_base_y = -0.45
        robot_yaw = 1.5708
        arm_offset_y = -0.031
        arm_world_x = robot_base_x - math.sin(robot_yaw) * arm_offset_y
        arm_world_y = robot_base_y + math.cos(robot_yaw) * arm_offset_y
        
        dx = self.target_position.x - arm_world_x
        dy = self.target_position.y - arm_world_y
        
        horizontal_dist = math.sqrt(dx**2 + dy**2)
        
        angle_to_target = math.atan2(dy, dx)
        theta1 = angle_to_target - robot_yaw
        
        while theta1 < -1.396263:
            theta1 += 2 * math.pi
        while theta1 > 3.490659:
            theta1 -= 2 * math.pi
        
        # 下降姿态：增加joint4使末端下降到抓取高度
        # 苹果高度约0.79m，需要末端下降到这个高度
        joint_angles = [
            theta1,      # joint1: 指向目标
            2.4,         # joint2: 大臂更前倾，往前伸
            0.0,         # joint3: 不旋转
            1.6,         # joint4: 肘部弯曲更多，使末端下降
            0.0,         # joint5: 腕部不旋转
            0.0,         # joint6: 腕部水平
            0.0          # joint7: 末端不旋转
        ]
        
        self._publish_joint_command(joint_angles, 'right')
        self.get_logger().info(f'📐 下降姿态: theta1={math.degrees(theta1):.1f}° 水平距离={horizontal_dist:.3f}m')
    
    def _publish_joint_command(self, joint_angles, arm='right'):
        """发布关节命令"""
        # 右臂关节限制
        joint_limits = [
            (-1.396263, 3.490659),   # joint1
            (-0.174533, 3.316125),   # joint2
            (-1.570796, 1.570796),   # joint3
            (0.0, 2.443461),         # joint4
            (-1.570796, 1.570796),   # joint5
            (-0.785398, 0.785398),   # joint6
            (-1.570796, 1.570796)    # joint7
        ]
        
        # 限制关节角度
        for i in range(len(joint_angles)):
            joint_angles[i] = max(joint_limits[i][0], min(joint_limits[i][1], joint_angles[i]))
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'openarm_{arm}_joint{i}' for i in range(1, 8)]
        msg.position = joint_angles
        self.pub_joint_cmd.publish(msg)
    
    def send_gripper_command(self, command: str):
        """发送夹爪控制命令"""
        msg = String()
        msg.data = command
        self.pub_gripper_cmd.publish(msg)
    
    def publish_grasp_status(self, status: str):
        """发布抓取状态"""
        msg = String()
        msg.data = status
        self.pub_grasp_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
