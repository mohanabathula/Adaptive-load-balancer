#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import json

class CarlaCarDistanceNode:
    def __init__(self):
        rospy.init_node('carla_car_distance_node', anonymous=True)

        # Parameters
        self.rgb_topic = rospy.get_param('~rgb_topic', '/carla/ego_vehicle/rgb_front/image')
        self.depth_topic = rospy.get_param('~depth_topic', '/carla/ego_vehicle/depth_front/image')
        self.model_path = rospy.get_param('~model_path', 'yolov5su.pt')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.4)
        self.resized_width = rospy.get_param('~resized_width', 640)
        self.resized_height = rospy.get_param('~resized_height', 480)

        # ROS Publisher for car detections with distance
        self.car_pub = rospy.Publisher('/carla/local_detection', String, queue_size=1)
        self.bridge = CvBridge()

        # Load YOLO model
        rospy.loginfo(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path).cuda()

        # Subscribe to RGB and Depth image topics
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)

        # Detection storage
        self.detection_results = []
        self.depth_image = None
        self.lock = threading.Lock()

        rospy.loginfo("CARLA Car Distance node initialized")

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (self.resized_width, self.resized_height))

            # Run YOLO detection
            results = self.model.predict(cv_image,verbose=False)[0]

            with self.lock:
                self.detection_results = []
                car_count = 0

                for r in results.boxes:
                    conf = float(r.conf[0])
                    cls_idx = int(r.cls[0])
                    label = results.names[cls_idx].lower()

                    # Only cars
                    if label != 'car' or conf < self.conf_threshold:
                        continue

                    bbox = r.xyxy[0].tolist()  # [x1, y1, x2, y2]
                    car_count += 1
                    unique_label = f"car_{car_count}"

                    self.detection_results.append({
                        'bbox': bbox,
                        'class': 'car',
                        'confidence': conf,
                        'unique_label': unique_label
                    })

            # Draw detections and distance if depth is available
            display_image = cv_image.copy()
            with self.lock:
                for det in self.detection_results:
                    x1, y1, x2, y2 = map(int, det['bbox'])
                    distance = None
                    if self.depth_image is not None:
                        # Get median depth in bounding box
                        bbox_depth = self.depth_image[y1:y2, x1:x2]
                        if bbox_depth.size > 0:
                            distance = float(np.median(bbox_depth))
                            det['distance'] = distance

                    # Draw box and distance
                    color = (255, 255, 255)
                    cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 1)
                    label_text = det['unique_label']
                    if distance:
                        label_text += f" {distance:.2f}m"
                    cv2.putText(display_image, label_text, (x1, y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Show image
            cv2.imshow("Car Detection & Distance", display_image)
            cv2.waitKey(1)

            # Publish detections
            self.car_pub.publish(String(data=json.dumps(self.detection_results)))

        except Exception as e:
            rospy.logwarn(f"RGB callback error: {e}")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert to float32 meters if needed
            if depth_image.dtype != np.float32:
                depth_image = depth_image.astype(np.float32)
            with self.lock:
                self.depth_image = depth_image
        except Exception as e:
            rospy.logwarn(f"Depth callback error: {e}")


if __name__ == '__main__':
    try:
        node = CarlaCarDistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CARLA Car Distance node terminated")
    finally:
        cv2.destroyAllWindows()

