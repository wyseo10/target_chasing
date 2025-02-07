import os
import cv2

class VideoRecorder:
    #resolution=(width, height)
    def __init__(self, output_path=None, fps=10, resolution=(162, 162)):
        package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
        
        self.output_path = output_path if output_path else os.path.join(package_path, "tc/results/output.webm")
        self.fps = fps
        self.resolution = resolution
        self.video_writer = None

        # results 폴더 생성
        if not os.path.exists('results'):
            os.makedirs('results')

    def init_writer(self):
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'VP90')
            self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, self.resolution)

            if not self.video_writer.isOpened():
                print("Error: Failed to open VideoWriter")

    def write_frame(self, frame):
        if self.video_writer is None:
            self.init_writer()
        if frame is not None:
            self.video_writer.write(frame)
        else:
            print("Warning: Frame is None, skipping writing")