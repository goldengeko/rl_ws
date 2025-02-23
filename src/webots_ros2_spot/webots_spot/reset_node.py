import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import numpy as np

class SpotResetNode(Node):
    def __init__(self):
        super().__init__('spot_reset_node')
        self.reset_service_client = self.create_client(Empty, '/Spot/reset_position')
        self.reset_position()

    def reset_position(self):
        if not self.reset_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Reset service not available')
            return
        request = Empty.Request()
        future = self.reset_service_client.call_async(request)
        future.add_done_callback(self.reset_response_callback)

    def reset_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Robot position reset successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to reset robot position: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SpotResetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()