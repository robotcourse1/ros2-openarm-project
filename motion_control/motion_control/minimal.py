import rclpy
from rclpy.node import Node

class MinimalMotionControl(Node):
    def __init__(self):
        super().__init__('motion_control_minimal')
        self.get_logger().info("Motion control node started.")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalMotionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
