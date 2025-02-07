import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import time
from .include.object_detector import ObjectDetector
from .include.video_recorder import VideoRecorder
from .include.aideck_streamer import AIDeckStreamer

class CenterPublisher(Node):

    def __init__(self):
        super().__init__('center_publisher')
        self.publisher_x = self.create_publisher(Int32, 'center_x', 10)
        self.publisher_y = self.create_publisher(Int32, 'center_y', 10)
        timer_period = 0.005  # 최대 10fps
        self.timer = self.create_timer(timer_period, self.timer_detecting_callback)

        self.detector = ObjectDetector()
        self.recorder = VideoRecorder()
        self.streamer = AIDeckStreamer()

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

        # max_box 검증 후 center_x, center_y 가져오기
        if isinstance(max_box, dict) and 'center_x' in max_box and 'center_y' in max_box:
            center_x = int(max_box['center_x'])
            center_y = int(max_box['center_y'])
        else:
            self.get_logger().warn("max_box is invalid or missing keys: 'center_x', 'center_y'.")
            return  # 오류가 발생하지 않도록 조기 종료

        # ROS 2 메시지 생성 및 퍼블리시
        msg_x = Int32()
        msg_x.data = center_x
        msg_y = Int32()
        msg_y.data = center_y

        self.publisher_x.publish(msg_x)
        self.publisher_y.publish(msg_y)

        self.get_logger().info(f'Publishing max_box center: {center_x}, {center_y}')

        # 화면 표시
        cam_center_x = color_img.shape[1] // 2
        cam_center_y = color_img.shape[0] // 2
        cv2.circle(color_img, (cam_center_x, cam_center_y), 2, (0, 0, 255), -1)

        # 검출된 박스 그리기
        self.detector.draw_box(color_img, max_box)
        self.recorder.write_frame(color_img)

        cv2.imshow("YOLO Detection", color_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CenterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
