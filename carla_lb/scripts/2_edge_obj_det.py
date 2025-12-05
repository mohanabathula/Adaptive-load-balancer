#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import requests
import time
import json
import numpy as np
import threading

class EdgeForwarder:
    def __init__(self):
        rospy.init_node('edge_forwarder', anonymous=True)

        # Parameters from launch file
        self.server_url = rospy.get_param('~server_url')          # Mandatory
        self.edge_timeout = rospy.get_param('~edge_timeout', 1.0)
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.image_topic = rospy.get_param('~image_topic')        # Mandatory
        self.applications = rospy.get_param('~applications', [
            'lane_detection',
            'collision_avoidance',
            'traffic_sign_detection',
            'traffic_light_detection'
        ])
        
        self.frame_id = 0
        self.edge_server_available = True
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Publisher for detection results
        self.obj_pub = rospy.Publisher('/carla/edge_detection', String, queue_size=1)
        
        # Detection results storage
        self.detection_results = []

        # Subscribe to image topic and depth
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/carla/ego_vehicle/depth_front/image", Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo(f"Subscribed to {self.image_topic}, forwarding to {self.server_url}")

    def image_callback(self, msg):
        """Convert ROS image to OpenCV and forward to edge server."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            for app in self.applications:
                self.forward_to_edge(cv_image.copy(), app)
        except Exception as e:
            rospy.logwarn(f"Failed to convert or forward image: {e}")

    def forward_to_edge(self, frame, app):
        """Send frame to edge server for processing."""
        try:
            image = cv2.resize(frame, (self.frame_width, self.frame_height))
            success, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                rospy.logwarn("Failed to encode image")
                return

            headers = {
                'Content-Type': 'application/octet-stream',
                'Frame-Width': str(self.frame_width),
                'Frame-Height': str(self.frame_height),
                'Client-Timestamp': str(time.time()),
                'Frame-Identifier': f"frame_{self.frame_id}",
                'detection-flag': app
            }
            self.frame_id += 1

            response = requests.post(
                self.server_url,
                data=buffer.tobytes(),
                headers=headers,
                timeout=self.edge_timeout
            )

            if response.status_code == 200:
                self.edge_server_available = True
                self.process_edge_response(response, image, app)
            else:
                rospy.logwarn(f"Edge server returned status {response.status_code} for {app}")
                self.edge_server_available = False

        except requests.exceptions.Timeout:
            rospy.logwarn(f"Edge request timed out for {app}")
            self.edge_server_available = False
        except requests.exceptions.RequestException as e:
            rospy.logwarn(f"Edge request failed for {app}: {e}")
            self.edge_server_available = False
        except Exception as e:
            rospy.logerr(f"Unexpected error in forward_to_edge ({app}): {e}")
            self.edge_server_available = False

    def process_edge_response(self, response, frame, app):
        """Process edge server response and draw results on frame."""
        try:
            result = response.json()
        except json.JSONDecodeError:
            rospy.logwarn("Edge response not valid JSON")
            return

        with self.lock:
            # ----- Object Detection -----
            if 'OBD_model' in result:
                obj_det = result['OBD_model']
                if isinstance(obj_det, str):
                    try:
                        obj_det = json.loads(obj_det)
                    except:
                        obj_det = {}
                self.detection_results = obj_det.get('detections', [])

                # Assign unique IDs per class
                class_count = {}
                for det in self.detection_results:
                    cls = det.get("class","unknown").lower()
                    if cls not in class_count:
                        class_count[cls] = 1
                    else:
                        class_count[cls] += 1
                    det['unique_label'] = f"{cls}_{class_count[cls]}"

                self.draw_objects(frame)
                cv2.imshow("Edge Detection View", frame)
                cv2.waitKey(1)

    def draw_objects(self, frame):
        """Draw detected objects on frame with distance if available."""
        for det in self.detection_results:
            obj_class = det.get("class","").lower()
            unique_label = det.get("unique_label", obj_class)
            if obj_class not in ['car', 'pedestrian']:
                continue
                
            bbox = det.get('bbox')
            if bbox and all(v is not None for v in bbox):
                x1, y1, x2, y2 = map(int, bbox)
                # Color coding: blue for cars, green for pedestrians
                color = (0, 0, 0) if obj_class == 'car' else (255, 255, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # Display unique label
                cv2.putText(frame, unique_label, (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # Display distance if available
                if 'distance' in det:
                    distance_text = f"{det['distance']:.2f}m"
                    cv2.putText(frame, distance_text, (x1, y2+15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    def depth_callback(self, msg):
        """Compute distance to detected objects using depth image."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth_image.dtype != np.float32:
                depth_image = depth_image.astype(np.float32)

            with self.lock:
                det_to_pub =[]
                for det in self.detection_results:
                    bbox = det.get("bbox")
                    unique_label = det.get("unique_label", "unknown")
                    obj_class = det.get("class", "").lower()
                    confidence = det.get("confidence", 1.0)
                    if obj_class not in ['car', 'pedestrian']:
                        continue
                    if bbox and all(v is not None for v in bbox):
                        x1, y1, x2, y2 = map(int, bbox)
                        bbox_depth = depth_image[y1:y2, x1:x2]

                        if bbox_depth.size == 0:
                            continue

                        distance = float(np.median(bbox_depth))
                        det['distance'] = distance
                        print(f"{unique_label} distance: {distance:.2f} meters")

                        # ---------- PUBLISH IN SAME FORMAT AS CarlaCarDistanceNode ----------
                        det_to_pub.append({
                            "bbox": bbox,
                            "class": obj_class,
                            "confidence": det.get("confidence", 1.0),
                            "unique_label": unique_label,
                            "distance": distance
                        })
                        self.obj_pub.publish(String(data=json.dumps(det_to_pub)))

        except Exception as e:
            rospy.logwarn(f"Depth callback error: {e}")


if __name__ == '__main__':
    try:
        node = EdgeForwarder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Edge forwarder node terminated")
    finally:
        cv2.destroyAllWindows()

