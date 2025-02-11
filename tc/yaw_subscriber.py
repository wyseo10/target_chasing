import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class YawSubscriber(Node):
    def __init__(self):
        super().__init__('yaw_subscriber')

        self.subscription = self.create_subscription(
            Twist,
            'yaw_plot',
            self.listener_callback,
            10
        )

        self.get_logger().info("Yaw Subscriber Initialized")

    def listener_callback(self, msg):
        yaw = msg.linear.z  # Yaw 값
        yaw_rate = msg.angular.z  # YawRate 값

        self.get_logger().info(f"Received Yaw: {yaw:.4f}, YawRate: {yaw_rate:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = YawSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
