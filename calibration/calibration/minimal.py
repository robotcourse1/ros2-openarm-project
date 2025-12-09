import rclpy
from rclpy.node import Node

class MinimalCalibration(Node):
    def __init__(self):
        super().__init__('calibration_minimal')
        self.get_logger().info("Calibration node started.")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
