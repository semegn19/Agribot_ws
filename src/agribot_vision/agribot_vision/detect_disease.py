#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os

# TF2 Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

os.environ['YOLO_OFFLINE'] = 'True'
from ultralytics import YOLO

class AgriBotVision(Node):
    def __init__(self):
        super().__init__('agribot_vision_node')
        
        # 1. Load YOLO model
        self.model = YOLO('/home/semegn/agribot_ws/src/agribot_vision/agribot_vision/best.pt')
        
        # 2. Setup TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 3. Subscriber
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.bridge = CvBridge()
        
        self.log_file = '/home/semegn/agribot_ws/farm_health_log.csv'
        self.last_log_time = 0.0
   
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w') as f:
                csv.writer(f).writerow(['X', 'Y', 'Status', 'Disease_Name', 'Confidence', 'Timestamp'])

        self.get_logger().info("Vision Node: Ready to classify and log farm health.")

    def get_coords(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            return None, None

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model.predict(source=frame, conf=0.3, imgsz=640, verbose=False)
        
        x_map, y_map = self.get_coords()

        # Keywords that indicate a problem
        unhealthy_keywords = [
            "scab", "rust", "gray", "spot", "blight", "mildew", 
            "virus", "mold", "rot", "bacterial", "mites"
        ]

        for r in results:
            for box in r.boxes:
                raw_label = self.model.names[int(box.cls[0])]
                label_lower = raw_label.lower()
                conf = float(box.conf[0])
                b = box.xyxy[0].cpu().numpy().astype(int)

                if any(k in label_lower for k in unhealthy_keywords):
                    color = (0, 0, 255)  # Red for Unhealthy
                    status = "Unhealthy"
                else:
                    color = (0, 255, 0)  # Green for Healthy
                    status = "Healthy"

                display_text = f"{raw_label} ({conf:.2f})"
                cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), color, 3)
                cv2.putText(frame, display_text, (b[0], b[1]-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # 5. Log to CSV
                if x_map is not None:
                    self.save_detection(x_map, y_map, status, raw_label, conf)

        cv2.imshow("AgriBot Vision System", frame)
        cv2.waitKey(1)

    def save_detection(self, x, y, status, disease_name, conf):
        now = self.get_clock().now().nanoseconds / 1e9
        # Log every 3 seconds to avoid duplicating the same plant constantly
        if (now - self.last_log_time) > 3.0:
            self.last_log_time = now
            self.get_logger().info(f"LOGGED: {disease_name} at ({x:.2f}, {y:.2f})")
            with open(self.log_file, 'a') as f:
                csv.writer(f).writerow([
                    round(x, 2), 
                    round(y, 2), 
                    status, 
                    disease_name, 
                    round(conf, 3), 
                    now
                ])

def main():
    rclpy.init()
    node = AgriBotVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
