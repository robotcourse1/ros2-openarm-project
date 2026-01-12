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
        
        # === 测试模式：直接发布已知物体位置 ===
        self.declare_parameter('test_mode', True)  # 设置为True跳过视觉检测
        self.test_mode = self.get_parameter('test_mode').value
        
        # === 1. 参数设置 ===
        self.target_color_lower = np.array([0, 100, 100])   # 红色HSV下限
        self.target_color_upper = np.array([10, 255, 255])  # 红色HSV上限
        
        # === 2. 手眼标定矩阵 ===
        self.T_base_cam = np.array([
            [ 0.0, -1.0,  0.0,  0.5],
            [-1.0,  0.0,  0.0,  0.0],
            [ 0.0,  0.0, -1.0,  0.8],
            [ 0.0,  0.0,  0.0,  1.0]
        ])

        # === 3. 通信接口 ===
        self.bridge = CvBridge()
        self.sub_color = self.create_subscription(Image, '/camera/image_raw', self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.pub_target = self.create_publisher(Point, '/target_pose', 10)
        
        self.latest_depth_img = None
        self.image_count = 0
        self.detection_count = 0
        
        # 测试模式：定时发布已知物体位置
        if self.test_mode:
            self.get_logger().info('🧪 测试模式已启动：将直接发布已知物体位置（苹果）')
            self.timer = self.create_timer(2.0, self.publish_test_target)
        else:
            self.get_logger().info('📷 真实视觉感知节点已启动! 等待图像...')

    def publish_test_target(self):
        """测试模式：直接发布苹果的已知位置"""
        point_msg = Point()
        point_msg.x = 0.85  # 苹果X坐标
        point_msg.y = -0.15  # 苹果Y坐标
        point_msg.z = 0.79  # 苹果Z坐标
        self.pub_target.publish(point_msg)
        self.get_logger().info(f'🍎 [测试模式] 发布苹果位置: ({point_msg.x:.2f}, {point_msg.y:.2f}, {point_msg.z:.2f})')

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')

    def color_callback(self, msg):
        if self.test_mode:
            return  # 测试模式下跳过图像处理
        
        self.image_count += 1
        
        # 每10帧打印一次日志，避免刷屏
        if self.image_count % 10 == 0:
            self.get_logger().info(f'📸 已接收 {self.image_count} 帧图像，检测到 {self.detection_count} 次物体')
        
        if self.latest_depth_img is None:
            return

        try:
            # 1. 图像转 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. 颜色识别 (HSV)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
            
            # 3. 找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                # 过滤太小的检测（噪声）
                if area < 100:
                    return
                
                x, y, w, h = cv2.boundingRect(c)
                u = int(x + w / 2)
                v = int(y + h / 2)
                
                # 4. 获取深度值
                depth = self.latest_depth_img[v, u]
                
                if np.isnan(depth) or depth <= 0:
                    return

                # 5. 像素 -> 相机坐标系
                fx = 554.25
                fy = 554.25
                cx = 320.0
                cy = 240.0
                
                z_c = depth
                x_c = (u - cx) * z_c / fx
                y_c = (v - cy) * z_c / fy
                
                # 6. 相机坐标系 -> 基座坐标系
                p_cam = np.array([x_c, y_c, z_c, 1.0])
                p_base = np.dot(self.T_base_cam, p_cam)
                
                # 7. 发布结果
                point_msg = Point()
                point_msg.x = float(p_base[0])
                point_msg.y = float(p_base[1])
                point_msg.z = float(p_base[2])
                self.pub_target.publish(point_msg)
                
                self.detection_count += 1
                self.get_logger().info(f'✅ 检测到物体! 像素:({u},{v}), 面积:{area:.0f}, 深度:{depth:.2f}m -> 基座:({point_msg.x:.2f}, {point_msg.y:.2f}, {point_msg.z:.2f})')

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
