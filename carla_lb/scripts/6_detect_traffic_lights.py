#!/usr/bin/env python3
"""
ROS Traffic Light Color & Arrow Detection
Subscribes to CARLA RGB camera and publishes traffic status
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


# ---------------- HELPER FUNCTIONS (UNCHANGED LOGIC) ---------------- #

def detect_arrow_direction(roi, color_mask):
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    cnt = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(cnt)
    if w == 0 or h == 0:
        return None

    lit = color_mask[y:y+h, x:x+w]
    aspect = w / h

    if 0.8 < aspect < 1.2:
        return None

    if aspect > 1.2:
        mid = w // 2
        left = np.sum(lit[:, :mid])
        right = np.sum(lit[:, mid:])
        if left + right == 0:
            return None
        return 'left' if left > right else 'right'

    return None


def analyze_traffic_light_color(image, box):
    x1, y1, x2, y2 = map(int, box)
    roi = image[y1:y2, x1:x2]
    if roi.size == 0:
        return 'unknown', None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    _, bright = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    if np.sum(bright) < 100:
        return 'unknown', None

    red1 = cv2.inRange(hsv, (0,70,50), (10,255,255))
    red2 = cv2.inRange(hsv, (160,70,50), (180,255,255))
    red = cv2.bitwise_and(cv2.bitwise_or(red1, red2), bright)

    yellow = cv2.bitwise_and(
        cv2.inRange(hsv, (15,80,80), (35,255,255)), bright)

    green = cv2.bitwise_and(
        cv2.inRange(hsv, (35,40,40), (95,255,255)), bright)

    counts = {
        'red': np.sum(red),
        'yellow': np.sum(yellow),
        'green': np.sum(green)
    }

    color = max(counts, key=counts.get)
    if counts[color] < 200:
        return 'unknown', None

    mask = {'red': red, 'yellow': yellow, 'green': green}[color]
    arrow = detect_arrow_direction(roi, mask)

    return color, arrow


# ---------------- ROS NODE ---------------- #

class TrafficLightColorROS:
    def __init__(self):
        rospy.init_node("traffic_light_color_detector")

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/carla/ego_vehicle/rgb_front/image"
        )
        self.model_path = rospy.get_param("~model", "yolo11s.pt")
        self.conf = rospy.get_param("~conf", 0.25)

        rospy.loginfo("Loading YOLO model...")
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        self.pub = rospy.Publisher(
            "/traffic_det",
            String,
            queue_size=10
        )

        rospy.loginfo("Traffic Light Color Detector READY")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model.predict(
            source=frame,
            conf=self.conf,
            classes=[9],
            verbose=False
        )

        detections = []

        for box in results[0].boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            color, arrow = analyze_traffic_light_color(frame, xyxy)
            if color == 'unknown':
                continue

            x1, y1, x2, y2 = map(int, xyxy)
            cx = (x1 + x2) / 2

            detections.append({
                'color': color,
                'arrow': arrow,
                'bbox': (x1, y1, x2, y2),
                'cx': cx
            })

        status_msg = "NO_TRAFFIC_LIGHT"

        if detections:
            img_cx = frame.shape[1] / 2
            main = min(detections, key=lambda d: abs(d['cx'] - img_cx))

            parts = [f"MAIN:{main['color'].upper()}"]

            for d in detections:
                if d['arrow']:
                    parts.append(f"ARROW:{d['arrow'].upper()}:{d['color'].upper()}")

            status_msg = " | ".join(parts)

            rospy.loginfo_throttle(1, f"🚦 {status_msg}")

            for d in detections:
                x1, y1, x2, y2 = d['bbox']
                color_map = {
                    'red': (0,0,255),
                    'yellow': (0,255,255),
                    'green': (0,255,0)
                }
                cv2.rectangle(frame, (x1,y1), (x2,y2),
                              color_map[d['color']], 2)
                label = d['color'].upper()
                if d['arrow']:
                    label += f" {d['arrow'].upper()}"
                cv2.putText(frame, label, (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255,255,255), 2)

        self.pub.publish(status_msg)

        cv2.imshow("Traffic Light Detection", frame)
        cv2.waitKey(1)


# ---------------- MAIN ---------------- #

if __name__ == "__main__":
    try:
        TrafficLightColorROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

