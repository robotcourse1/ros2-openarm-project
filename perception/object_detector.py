import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealVisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # === 1. 参数设置 ===
        self.target_color_lower = np.array([0, 100, 100])   # 红色HSV下限 (根据实际物体调整)
        self.target_color_upper = np.array([10, 255, 255])  # 红色HSV上限
        
        # === 2. 手眼标定矩阵 (来自 Member B 的标定结果) ===
        # T_base_cam: 相机在基座坐标系下的位置
        # 这里使用你之前 URDF 设置的: x=0.5, z=0.8, 俯视90度
        self.T_base_cam = np.array([
            [ 0.0, -1.0,  0.0,  0.5],
            [-1.0,  0.0,  0.0,  0.0],
            [ 0.0,  0.0, -1.0,  0.8],
            [ 0.0,  0.0,  0.0,  1.0]
        ])

        # === 3. 通信接口 ===
        self.bridge = CvBridge()
        # 订阅彩色图和深度图
        self.sub_color = self.create_subscription(Image, '/camera/image_raw', self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        # 发布物体位置给 Member C
        self.pub_target = self.create_publisher(Point, '/target_pose', 10)
        
        self.latest_depth_img = None
        self.get_logger().info('【Member B】真实视觉感知节点已启动! 等待图像...')

    def depth_callback(self, msg):
        # 实时保存最新的深度图
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')

    def color_callback(self, msg):
        if self.latest_depth_img is None:
            return # 等待深度图到位

        try:
            # 1. 图像转 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. 颜色识别 (HSV)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
            
            # 3. 找轮廓 (寻找最大的红色物体)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)
                
                # 计算中心点 (u, v)
                u = int(x + w / 2)
                v = int(y + h / 2)
                
                # 4. 获取深度值 (距离)
                # 注意：深度图可能有噪声(NaN或0)，这里简单取中心点
                # 生产环境通常取一个小区域的平均值
                depth = self.latest_depth_img[v, u]
                
                # 如果是 RealSense，单位通常是毫米，仿真里通常是米。
                # 假设 Gazebo 输出的是米 (float32)
                if np.isnan(depth) or depth <= 0:
                    return

                # 5. 像素 -> 相机坐标系 (Pinhole Model)
                # 简化参数: 640x480分辨率, 60度FOV -> 焦距 fx,fy ≈ 554
                fx = 554.25
                fy = 554.25
                cx = 320.0
                cy = 240.0
                
                z_c = depth
                x_c = (u - cx) * z_c / fx
                y_c = (v - cy) * z_c / fy
                
                # 6. 相机坐标系 -> 机械臂基座坐标系 (AX=XB)
                p_cam = np.array([x_c, y_c, z_c, 1.0])
                p_base = np.dot(self.T_base_cam, p_cam)
                
                # 7. 发布结果
                point_msg = Point()
                point_msg.x = float(p_base[0])
                point_msg.y = float(p_base[1])
                point_msg.z = float(p_base[2])
                self.pub_target.publish(point_msg)
                
                # 打印日志 (方便你截图交差)
                self.get_logger().info(f'检测到物体! 像素:({u},{v}) -> 深度:{depth:.2f}m -> 基座坐标:({point_msg.x:.2f}, {point_msg.y:.2f}, {point_msg.z:.2f})')

        except Exception as e:
            self.get_logger().error(f'图像处理出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RealVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
