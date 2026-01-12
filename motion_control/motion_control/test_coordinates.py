#!/usr/bin/env python3
"""
坐标转换测试工具 - 验证世界坐标到机械臂坐标的转换
"""

import math

def test_coordinate_transform():
    """测试坐标转换"""
    
    print("=" * 80)
    print("坐标系统配置：")
    print("-" * 80)
    
    # 机器人基座在世界坐标系中的位置和姿态
    robot_base_x = 0.8
    robot_base_y = -0.45
    robot_base_z = 0.35
    robot_yaw = 1.5708  # 90度，绕Z轴旋转
    
    print(f"机器人基座（世界坐标）: ({robot_base_x}, {robot_base_y}, {robot_base_z})")
    print(f"机器人基座旋转（欧拉角）: (0, 0, {math.degrees(robot_yaw):.1f}°)")
    
    # 左臂基座相对于机器人基座的偏移（在机器人坐标系中）
    left_arm_offset_x = 0.0
    left_arm_offset_y = 0.031
    left_arm_offset_z = 0.698
    
    print(f"左臂偏移（机器人坐标系）: ({left_arm_offset_x}, {left_arm_offset_y}, {left_arm_offset_z})")
    
    # 左臂基座在世界坐标系中的位置
    # 机器人旋转了90度：机器人的+Y -> 世界的+X，机器人的-X -> 世界的+Y
    arm_world_x = robot_base_x + math.cos(robot_yaw) * left_arm_offset_x - math.sin(robot_yaw) * left_arm_offset_y
    arm_world_y = robot_base_y + math.sin(robot_yaw) * left_arm_offset_x + math.cos(robot_yaw) * left_arm_offset_y
    arm_world_z = robot_base_z + left_arm_offset_z
    
    print(f"左臂基座（世界坐标）: ({arm_world_x:.3f}, {arm_world_y:.3f}, {arm_world_z:.3f})")
    
    print("\n" + "=" * 80)
    print("物体位置测试：")
    print("-" * 80)
    
    # 测试物体位置
    objects = [
        ("苹果", 0.85, -0.15, 0.79),
        ("香蕉", 0.75, -0.15, 0.79),
    ]
    
    for name, obj_x, obj_y, obj_z in objects:
        print(f"\n{name}位置（世界坐标）: ({obj_x}, {obj_y}, {obj_z})")
        
        # 计算相对于左臂基座的偏移（世界坐标系）
        dx_world = obj_x - arm_world_x
        dy_world = obj_y - arm_world_y
        dz_world = obj_z - arm_world_z
        
        print(f"  相对左臂基座（世界坐标）: dx={dx_world:.3f}, dy={dy_world:.3f}, dz={dz_world:.3f}")
        
        # 转换到机器人坐标系
        dx_robot = math.cos(-robot_yaw) * dx_world - math.sin(-robot_yaw) * dy_world
        dy_robot = math.sin(-robot_yaw) * dx_world + math.cos(-robot_yaw) * dy_world
        dz_robot = dz_world
        
        print(f"  相对左臂基座（机器人坐标）: dx={dx_robot:.3f}, dy={dy_robot:.3f}, dz={dz_robot:.3f}")
        
        # 距离
        distance = math.sqrt(dx_world**2 + dy_world**2 + dz_world**2)
        print(f"  直线距离: {distance:.3f}m")
        
        # Joint1应该的角度（在世界坐标系中）
        angle_world = math.atan2(dy_world, dx_world)
        # 转换到机器人坐标系
        angle_robot = angle_world - robot_yaw
        # 转换到左臂坐标系（考虑左臂的-90度X轴旋转）
        angle_arm = angle_robot
        
        print(f"  Joint1角度建议: 世界={math.degrees(angle_world):.1f}°, 机器人={math.degrees(angle_robot):.1f}°, 左臂={math.degrees(angle_arm):.1f}°")
        
        # 检查是否在工作空间内
        max_reach = 0.6  # 假设最大臂展
        if distance > max_reach:
            print(f"  ⚠️ 警告：距离 {distance:.3f}m 超出最大臂展 {max_reach}m")
        else:
            print(f"  ✅ 在工作空间内")
    
    print("\n" + "=" * 80)


if __name__ == '__main__':
    test_coordinate_transform()
