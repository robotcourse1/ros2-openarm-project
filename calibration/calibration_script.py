import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import yaml
import os
import time
from scipy.spatial.transform import Rotation as R

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.get_logger().info("标定节点启动: 正在监听 TF 数据...")
        
        # 1. 初始化 TF 监听器 (任务 2)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建定时器，1秒后执行一次标定流程
        self.timer = self.create_timer(1.0, self.perform_calibration)

    def perform_calibration(self):
        try:
            # 尝试获取 base_link 到 camera_link 的变换
            # 真实标定中，这里应该是获取多组 (Hand, Eye) 数据
            # 在仿真/简化版中，我们直接读取当前的 TF 树作为"真值"或"初始值"
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 
                'camera_link', 
                rclpy.time.Time()
            )
            
            self.get_logger().info(f"获取到 TF 数据: Translation={trans.transform.translation}")
            
            # 2. 提取数据
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            tz = trans.transform.translation.z
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            # 3. 转换矩阵 (使用 scipy)
            r = R.from_quat([qx, qy, qz, qw])
            rotation_matrix = r.as_matrix()
            
            T_base_cam = np.eye(4)
            T_base_cam[:3, :3] = rotation_matrix
            T_base_cam[:3, 3] = [tx, ty, tz]
            
            # 4. 保存结果 (YAML)
            self.save_to_yaml(T_base_cam, [tx, ty, tz], r.as_euler('xyz', degrees=False))
            
            # 任务完成，退出节点
            self.get_logger().info("标定完成，节点即将退出。")
            raise SystemExit

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("等待 TF 数据中...")

    def save_to_yaml(self, matrix, trans, euler):
        # 确保目录存在
        config_dir = os.path.join(os.getcwd(), 'config')
        if not os.path.exists(config_dir):
            try:
                os.makedirs(config_dir)
            except OSError:
                pass # 如果创建失败就在当前目录存

        save_path = 'calibration/hand_eye_result.yaml'
        
        data = {
            'calibration_time': time.strftime("%Y-%m-%d %H:%M:%S"),
            'method': 'tf_listener_automatic',
            'transform_matrix': matrix.flatten().tolist(),
            'translation': {'x': trans[0], 'y': trans[1], 'z': trans[2]},
            'rotation_euler': {'r': float(euler[0]), 'p': float(euler[1]), 'y': float(euler[2])}
        }
        
        with open(save_path, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"标定文件已保存至: {save_path}")

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
