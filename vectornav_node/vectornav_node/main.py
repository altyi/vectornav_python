import rclpy
from vectornav_node.driver import VectorNavNode

def main(args=None):
    rclpy.init(args=args)
    node = VectorNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()