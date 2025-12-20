import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import time

# === Member B 核心工作：标定矩阵 ===
# 这里的数值对应你在 URDF 里设置的 <origin xyz="0.5 0 0.8" ...>
# 如果以后你改了 URDF，这里也要跟着改
T_base_cam = np.array([
    [0, -1,  0,  0.5],  # 旋转 + X平移
    [-1, 0,  0,  0.0],  # 旋转 + Y平移
    [0,  0, -1,  0.8],  # 旋转 + Z平移
    [0,  0,  0,  1.0]
])

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # 创建发布者：把物体坐标发给 Member C (Topic: /target_pose)
        self.publisher_ = self.create_publisher(Point, '/target_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 每秒执行一次
        self.get_logger().info('【Member B】视觉感知节点已启动！正在寻找物体...')

    def timer_callback(self):
        # 1. 模拟视觉识别 (OpenCV)
        # 假设我们在图像中心 (320, 240) 识别到了一个红苹果
        # 深度相机测出距离是 0.75 米
        u, v, depth = 320, 240, 0.75 

        # 2. 模拟计算 (相机坐标系)
        # 假设相机内参 fx=600
        x_c = (u - 320) * depth / 600.0  # 结果是 0
        y_c = (v - 240) * depth / 600.0  # 结果是 0
        z_c = depth                      # 结果是 0.75

        # 3. 核心步骤：坐标变换 (相机 -> 基座)
        p_cam = np.array([x_c, y_c, z_c, 1.0]) # 齐次坐标
        p_base = np.dot(T_base_cam, p_cam)     # 矩阵乘法

        # 4. 打包消息并发布
        msg = Point()
        msg.x = float(p_base[0])
        msg.y = float(p_base[1])
        msg.z = float(p_base[2])

        self.publisher_.publish(msg)

        # 打印日志给队友看
        print(f"[视觉输出] 发现物体! 坐标(Base系): X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
