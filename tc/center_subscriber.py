import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CenterSubscriber(Node):
    def __init__ (self):
        super().__init__('center_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'max_box_center',
            self.listener_callback,
            10
        )
        self.subscription
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribing : x={msg.x:.2f}, y={msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CenterSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()