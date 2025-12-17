import rclpy
from rclpy.node import Node

class MinimalPerception(Node):
    def __init__(self):
        super().__init__('perception_minimal')
        self.get_logger().info("Perception node started.")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
