import socket
import struct
import numpy as np
import cv2
import argparse

class AIDeckStreamer:
    def __init__(self, cam_width=162, cam_height=122):
        # AI-deck IP/port 불러오기
        parser = argparse.ArgumentParser(description='Connect to AI-deck streamer')
        parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
        parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
        parser.add_argument('--save', action='store_true', help="Save streamed images")
        args = parser.parse_args()

        self.ip = args.n
        self.port = args.p
        self.cam_width = cam_width
        self.cam_height = cam_height
        self.client_socket = None

    def connect(self):
        print(f"Connecting to socket on {self.ip}:{self.port}...")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        print("Socket Connected")

    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data
    
    # Getting color_img from AIdeck
    def get_img(self):
        packetInfoRaw = self.rx_bytes(4)
        [length, routing, function] = struct.unpack('HBB', packetInfoRaw)

        imgHeader = self.rx_bytes(length - 2)
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        if magic != 0xBC:
            return None
        
        img_stream = bytearray()
        while len(img_stream) < size:
            packetInfoRaw = self.rx_bytes(4)
            [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
            img_stream.extend(self.rx_bytes(length - 2))
            
        nparr = np.frombuffer(img_stream, np.uint8)
        color_img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)

        # JPEG color_img.shape check
        if color_img is None:
            print("[ERROR] JPEG 디코딩 실패")
            return None
        if len(color_img.shape) == 2:  # Grayscale image
            color_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)
        if color_img.shape[2] != 3:
            print(f"[ERROR] 이미지가 3채널이 아님: {color_img.shape}")
            return None
        #Checking ends
        return color_img