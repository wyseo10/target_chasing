import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import cv2
import time
import os
from tc.include.object_detector import ObjectDetector
from tc.include.video_recorder import VideoRecorder
from tc.include.aideck_streamer import AIDeckStreamer

class CenterPublisher(Node):

    def __init__(self):
        super().__init__('center_publisher')
        self.publisher_ = self.create_publisher(PointStamped, 'max_box_center', 10)
        timer_period = 0.005  # 최대 10fps
        self.timer = self.create_timer(timer_period, self.timer_detecting_callback)

        self.detector = ObjectDetector()
        self.recorder = VideoRecorder()
        self.streamer = AIDeckStreamer()
        #self.streamer.update_resolution()

        self.streamer.connect()
        self.count = 0
        self.start = time.time()

    def timer_detecting_callback(self):
        color_img = self.streamer.get_frame()
        if color_img is None:
            self.get_logger().warn("No frame received from AIDeck.")
            return

        self.count += 1
        mean_time_per_image = (time.time() - self.start) / self.count
        self.get_logger().info(f"Frame rate: {1 / mean_time_per_image:.2f} fps")

        # YOLO 감지 수행
        max_box = self.detector.detect(color_img)
        center_x, center_y = color_img.shape[1] // 2, color_img.shape[0] // 2

        if isinstance(max_box, dict) and 'center_x' in max_box and 'center_y' in max_box:
            center_x = int(max_box['center_x'])
            center_y = int(max_box['center_y'])
        else:
            self.get_logger().warn("⚠️ No person detected. Displaying only the camera feed.")

        # ROS 2 메시지 생성 및 퍼블리시
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(center_x)
        msg.point.y = float(center_y)
        msg.point.z = 0.0

        self.publisher_.publish(msg)

        self.get_logger().info(f'Publishing max_box center: {center_x}, {center_y}')

        try:
            cam_center_x = color_img.shape[1] // 2
            cam_center_y = color_img.shape[0] // 2
            cv2.circle(color_img, (cam_center_x, cam_center_y), 2, (255, 0, 0), -1)  # 중앙점 표시

            if max_box.get("found", False):
                self.detector.draw_box(color_img, max_box)

            self.recorder.write_frame(color_img)

            cv2.imshow("YOLO Detection", color_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CenterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()