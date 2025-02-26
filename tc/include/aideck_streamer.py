import socket
import struct
import numpy as np
import cv2
import argparse

class AIDeckStreamer:
    def __init__(self, flag_jpeg_encoder=True):
        # AI-deck IP/port 불러오기
        parser = argparse.ArgumentParser(description='Connect to AI-deck streamer')
        parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
        parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
        parser.add_argument('--save', action='store_true', help="Save streamed images")
        args = parser.parse_args()

        self.ip = args.n
        self.port = args.p
        self.flag_jpeg_encoder = flag_jpeg_encoder
        self.client_socket = None

        if self.flag_jpeg_encoder:
            self.cam_width, self.cam_height = 162, 122 #for cf07
        else:
            self.cam_width, self.cam_height = 162, 162 #for cf04
        print(f"[INFO] Encoder Mode : {'JPEG' if self.flag_jpeg_encoder else 'RAW'}")

    def connect(self):
        print(f"Connecting to socket on {self.ip}:{self.port}...")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        print("Socket Connected")
        
    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size - len(data)))
        return data 

    def get_frame(self):
        """ 수신된 프레임을 JPEG 또는 RAW로 처리 """
        packetInfoRaw = self.rx_bytes(4)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

        img_header = self.rx_bytes(length - 2)
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', img_header)

        if magic != 0xBC:
            return None
        
        img_stream = bytearray()
        while len(img_stream) < size:
            packetInfoRaw = self.rx_bytes(4)
            [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
            img_stream.extend(self.rx_bytes(length - 2))

        if self.flag_jpeg_encoder:
            return self.process_jpeg_image(img_stream)
        else:
            return self.process_raw_image(img_stream)

    def process_jpeg_image(self, img_stream):
        """ JPEG 이미지를 디코딩 및 변환 """
        nparr = np.frombuffer(img_stream, np.uint8)
        color_img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)

        if color_img is None:
            print("[ERROR] JPEG 디코딩 실패")
            return None

        if len(color_img.shape) == 2:  # Grayscale 이미지인 경우
            color_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)

        if color_img.shape[2] != 3:
            print(f"[ERROR] 이미지가 3채널이 아님: {color_img.shape}")
            return None

        return color_img

    def process_raw_image(self, img_stream):
        """ RAW 이미지를 Bayer -> BGR로 변환 """
        bayer_img = np.frombuffer(img_stream, dtype=np.uint8)
        bayer_img.shape = (self.cam_height, self.cam_width)
        color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
        return color_img
