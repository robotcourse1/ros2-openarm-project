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
        
        # 1. 动态内参 (先给默认值防崩，收到消息后更新)
        self.fx = 554.25
        self.fy = 554.25
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False

        # 2. 订阅 Camera Info 获取真实内参
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # 3. 颜色阈值 (红色)
        self.target_color_lower = np.array([0, 100, 100])
        self.target_color_upper = np.array([10, 255, 255])
        
        # 4. 手眼标定矩阵 (Eye-to-Hand)
        self.T_base_cam = np.array([
            [ 0.0, -1.0,  0.0,  0.5],
            [-1.0,  0.0,  0.0,  0.0],
            [ 0.0,  0.0, -1.0,  0.8],
            [ 0.0,  0.0,  0.0,  1.0]
        ])

        self.bridge = CvBridge()
        self.sub_color = self.create_subscription(Image, '/camera/image_raw', self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.pub_target = self.create_publisher(Point, '/target_pose', 10)
        
        self.latest_depth_img = None
        self.get_logger().info('视觉节点启动: 等待相机参数...')

    def info_callback(self, msg):
        if not self.camera_info_received:
            # K矩阵: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(f'已获取真实相机内参: fx={self.fx:.1f}, cx={self.cx:.1f}')

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度图错误: {e}')

    def color_callback(self, msg):
        if self.latest_depth_img is None: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)
                u, v = int(x + w/2), int(y + h/2)
                
                depth = self.latest_depth_img[v, u]
                if np.isnan(depth) or depth <= 0: return

                # 使用动态内参计算
                z_c = depth
                x_c = (u - self.cx) * z_c / self.fx
                y_c = (v - self.cy) * z_c / self.fy
                
                p_cam = np.array([x_c, y_c, z_c, 1.0])
                p_base = np.dot(self.T_base_cam, p_cam)
                
                point_msg = Point()
                point_msg.x, point_msg.y, point_msg.z = float(p_base[0]), float(p_base[1]), float(p_base[2])
                self.pub_target.publish(point_msg)

        except Exception as e:
            self.get_logger().error(f'处理错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RealVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
