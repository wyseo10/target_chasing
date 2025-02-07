from ultralytics import YOLO
from collections import deque
import cv2
import os

class MovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, new_value):
        self.values.append(new_value)
        return self.calculate_average()

    def calculate_average(self):
        return sum(self.values) / len(self.values) if self.values else 0
    
    def get_stabilized_value(self):
       return self.calculate_average()

class ObjectDetector:
  def __init__(self, model_path=None, min_conf=0.5, window_size = 10):
      package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
      model_path = os.path.join(package_path, "tc/include/yolo11n.pt")

      self.model_path = model_path if model_path else os.path.join(package_path, "tc/include/yolo11n.pt")
      self.model = YOLO(model_path)
      self.min_confidence = min_conf

      self.moving_avg_x = MovingAverage(window_size)
      self.moving_avg_y = MovingAverage(window_size)
      self.moving_avg_w = MovingAverage(window_size)
      self.moving_avg_h = MovingAverage(window_size)
    
  def detect(self, image):
    results = self.model(image)[0] 
    max_box = {
      "confidence": 0.0,"found": False
      }
     
    for box in results.boxes:
        box_center_x, box_center_y, width, height = box.xywh[0]
        confidence = box.conf[0]
        class_id = int(box.cls[0])

        if class_id == 0 and confidence > self.min_confidence and confidence > max_box["confidence"]:
              max_box.update({
                "center_x": box_center_x,
                "center_y": box_center_y,
                "width": width,
                "height": height,
                "confidence": confidence,
                "class_name": self.model.names[class_id],
                "found": True
              })

    return self.stabilize(max_box)

  def stabilize(self, max_box):
    if max_box["found"]:
      max_box["center_x"] = self.moving_avg_x.update(max_box["center_x"])
      max_box["center_y"] = self.moving_avg_y.update(max_box["center_y"])
      max_box["width"] = self.moving_avg_w.update(max_box["width"])
      max_box["height"] = self.moving_avg_h.update(max_box["height"])
    return max_box
  
  def draw_box(self, img, max_box):
    if not max_box.get("found", False):
       #print("Any object detected.")
       return
    
    x1 = int(max_box["center_x"] - (max_box["width"] / 2))
    x2 = int(max_box["center_x"] + (max_box["width"] / 2))
    y1 = int(max_box["center_y"] - (max_box["height"] / 2))
    y2 = int(max_box["center_y"] + (max_box["height"] / 2))
        
    cv2.circle(img, (int(max_box["center_x"]), int(max_box["center_y"])), 2,(0,0,255),-1)
    cv2.rectangle(img, (x1, y1),(x2, y2), (0, 255, 0), 2)
    cv2.putText(img,f"{max_box['class_name']} {max_box['confidence']:.2f}",
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)