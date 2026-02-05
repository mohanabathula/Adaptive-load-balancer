#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import signal
import rospy
import time
import csv
import base64
import json
import threading
import numpy as np
import requests
from std_msgs.msg import String , Float64
from sensor_msgs.msg import Image
from lb_inputs import get_usage_percent, get_network_bandwidth, get_application_table
from ultralytics import YOLO


CSV_FILE = "/media/root/b9f388cb-81d8-4121-93fb-ad514e324552/vidyav/main/alb_edge_e2e.csv"
ARTIFICIAL_DELAY_MS = 0  # simulated delay in ms

# ANSI color codes
class Color:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'

class LoadBalancerNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.depth_image = None
        self.display_frame = None
        self.detection_results = []
        self.lock = threading.Lock()
        self.is_running = True
        self.t1 = None
        self.edge_e2e = None
        self.frame_ts = {}   # frame_id -> frame_rx_time


        # Handle signals
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)
        

        # ROS Publishers
        self.lane_img_pub = rospy.Publisher('/load_balancer/lane_image', Image, queue_size=10)
        self.yolo_img_pub = rospy.Publisher('/load_balancer/yolo_image', Image, queue_size=10)
        self.binary_img_pub = rospy.Publisher("/load_balancer/lane_binary_image", Image, queue_size=10)
        self.result_img_pub = rospy.Publisher("/load_balancer/lane_result_image", Image, queue_size=10)
        self.lane_pub = rospy.Publisher("/load_balancer/lane_detection", String, queue_size=10)
        self.obj_pub = rospy.Publisher("/load_balancer/object_detections", String, queue_size=10)
        self.traffic_pub = rospy.Publisher("/load_balancer/traffic_sign_detections", String, queue_size=10)
        
        self.cmd_pub = rospy.Publisher("/time_cmd",Float64, queue_size=1)
     

        self.artificial_delay = rospy.get_param('~artificial_delay_ms', ARTIFICIAL_DELAY_MS)

        # ROS Subscribers
        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, self.image_callback)
        rospy.Subscriber('/carla/ego_vehicle/depth_front/image', Image, self.depth_callback)
        # Edge server
        self.server_url = "http://192.168.20.16:30052/processimage"
        self.frame_width = 640
        self.frame_height = 480
        self.frame_id = 0
        self.edge_timeout = 3.0
        self.resource_threshold = 10
        self.bandwidth_high_threshold = 10
        self.bandwidth_low_threshold = 5
        self.edge_server_available = True
        self.last_edge_check = 0
        self.edge_check_interval = 5.0

        # CSV file
        self.csv_file = open(CSV_FILE, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "edge_e2e"])


        # Start display thread
        threading.Thread(target=self.display_loop, daemon=True).start()

        rospy.loginfo("LoadBalancerNode initialized.")
        self.run()

    def shutdown(self, signum, frame):
        self.is_running = False
        rospy.loginfo(Color.RED + "Shutting down LoadBalancerNode..." + Color.RESET)
        rospy.signal_shutdown("Signal received")
        cv2.destroyAllWindows()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.is_running:
            if self.image is not None:
                self.handle_frame(self.image)
            rate.sleep()

    def image_callback(self, ros_image):
        try:
            frame_rx_time = time.time()   # T0 (ground truth)
            self.frame_rx_time = frame_rx_time
       
            self.cmd_pub.publish(Float64(data=frame_rx_time))

            self.cmd_pub.publish(Float64(data=self.t1))
            # Artificial delay injection
            if self.artificial_delay > 0:
                time.sleep(self.artificial_delay / 1000.0)

            if ros_image.encoding in ['bgra8', 'bgr8']:
                img_np = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, -1))
                self.image = img_np[:, :, :3]
                
            else:
                rospy.logwarn(f"Unsupported encoding: {ros_image.encoding}")
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def depth_callback(self, ros_image):
        try:
            depth_np = np.frombuffer(ros_image.data, dtype=np.float32).reshape((ros_image.height, ros_image.width))
            self.depth_image = depth_np
            # Update distances for current detections
            self.update_distances()
        except Exception as e:
            rospy.logwarn(f"Depth callback error: {e}")

    def update_distances(self):
        with self.lock:
            updated_detections = []
            for det in self.detection_results:
                bbox = det.get("bbox")
                if bbox and self.depth_image is not None:
                    x1, y1, x2, y2 = map(int, bbox)
                    patch = self.depth_image[y1:y2, x1:x2]
                    if patch.size > 0:
                        distance = float(np.median(patch))
                        det['distance'] = distance
                updated_detections.append(det)
            # Publish updated detection with distances
            self.obj_pub.publish(String(data=json.dumps(updated_detections)))
            self.annotate_frame()

    def handle_frame(self, frame):
        try:
            usage = get_usage_percent()
            bandwidth = get_network_bandwidth()
            edge_available = self.check_edge_server_available()

            applications = ['collision_avoidance']  # 'lane_detection','collision_avoidance','traffic_light_detection'

            for app in applications:
                r = get_application_table(app)
                if r is None:
                    rospy.logwarn(f"Application '{app}' not found in table! Using onboard.")
                    self.forward_to_onboard(app, frame)
                    continue

                latency = r.get("latency_sensitivity")
                accuracy = r.get("accuracy_priority")

                loc = self.decide_processing_location(
                    latency, accuracy,
                    usage['cpu_percent'], usage['ram_percent'], usage['gpu_percent'],
                    bandwidth['upload_Mbps'], bandwidth['download_Mbps'],
                    edge_available
                )

                if loc=="onboard":
                    print(Color.YELLOW + f"{app}: Processing onboard" + Color.RESET)
                    self.forward_to_onboard(app, frame)
                else:
                    print(Color.GREEN + f"{app}: Offloading to edge" + Color.RESET)
                    self.forward_to_edge(frame, app)

        except Exception as e:
            rospy.logerr(f"Error in handle_frame: {e}")

    def decide_processing_location(self, latency_sensitivity, accuracy_priority, cpu, ram, gpu, up, down, edge_available):
        #print(latency_sensitivity, accuracy_priority, cpu, ram, gpu, up, down, edge_available)
        onboard_ok = (cpu < self.resource_threshold and gpu < self.resource_threshold)
        bandwidth_ok = (up>=self.bandwidth_high_threshold)
        if not edge_available:
            return "onboard"
        if latency_sensitivity=="high":
            if onboard_ok: return "onboard"
            return "offboard" if bandwidth_ok else "onboard"
        else:
            if accuracy_priority=="high": return "offboard" if bandwidth_ok else "onboard"
            return "offboard" if bandwidth_ok else "lower_resolution"

    def check_edge_server_available(self):
        now = time.time()
        if now - self.last_edge_check < self.edge_check_interval:
            return self.edge_server_available
        try:
            resp = requests.head(self.server_url, timeout=2.0)
            self.edge_server_available = resp.status_code < 500
        except:
            self.edge_server_available = False
        self.last_edge_check = now
        return self.edge_server_available

    def lower_resolution(self, frame, scale=0.5):
        h,w = frame.shape[:2]
        return cv2.resize(frame, (int(w*scale), int(h*scale)))

    def forward_to_edge(self, frame, app):
        try:
            frame_id = self.frame_id
            send_time = self.frame_rx_time   # use camera receive time
            self.frame_ts[frame_id] = send_time

            img_resized = cv2.resize(frame, (self.frame_width, self.frame_height))
            success, buffer = cv2.imencode('.jpg', img_resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                rospy.logwarn("Failed to encode image")
                return

            headers = {'Content-Type':'application/octet-stream',
                       'Frame-Width':str(self.frame_width),
                       'Frame-Height':str(self.frame_height),
                       'Frame-Identifier':f"frame_{self.frame_id}",
                       'detection-flag': app,
                       'Client-Timestamp':str(send_time)}
            self.frame_id += 1

            resp = requests.post(self.server_url, data=buffer.tobytes(), headers=headers, timeout=self.edge_timeout)
            if resp.status_code==200:
                self.process_edge_response(resp, app)
            else:
                rospy.logwarn(f"Edge returned {resp.status_code}")
        except Exception as e:
            rospy.logwarn(f"Edge request failed: {e}")

    def process_edge_response(self, resp, app):
        try:
            recv_time = time.time()  # This is T1
            frame_id_header = resp.request.headers.get('Frame-Identifier')
            frame_id = int(frame_id_header.split("_")[1])
            send_time = self.frame_ts.pop(frame_id)

            edge_e2e_ms = (recv_time - send_time) * 1000.0
            print(f"Cloud E2E: {edge_e2e_ms:.2f} ms")
            self.csv_writer.writerow([time.time(), edge_e2e_ms])
            result = resp.json()
        except:
            rospy.logerr("Invalid JSON from edge")
            return
        with self.lock:
            # Lane
            if 'Lane_detection_model' in result:
                lane_data = result['Lane_detection_model']
                if isinstance(lane_data,str): lane_data=json.loads(lane_data)
                self.lane_data = lane_data
                self.lane_pub.publish(String(data=json.dumps(lane_data)))
                print(Color.GREEN+"Lane data published from edge"+Color.RESET)

            # Object detection
            if 'OBD_model' in result:
                obd_data = result['OBD_model']
                if isinstance(obd_data,str): obd_data=json.loads(obd_data)
                self.detection_results = obd_data.get('detections', [])
                # Assign unique labels
                count_dict = {}
                for det in self.detection_results:
                    cls = det.get("class","unknown").lower()
                    count_dict[cls] = count_dict.get(cls,0)+1
                    det['unique_label'] = f"{cls}_{count_dict[cls]}"
                # Publish detection immediately
                self.obj_pub.publish(String(data=json.dumps(self.detection_results)))
                self.annotate_frame()
                print(Color.GREEN+"Object detection published from edge"+Color.RESET)
                

            # Traffic signs
            if 'TrafficSign_model' in result:
                ts_data = result['TrafficSign_model']
                if isinstance(ts_data,str): ts_data=json.loads(ts_data)
                self.traffic_sign_results = ts_data.get('detections',[])
                self.traffic_pub.publish(String(data=json.dumps(self.traffic_sign_results)))
                print(Color.GREEN+"Traffic signs published from edge /n"+Color.RESET)

            
            
   
    def annotate_frame(self):
        if self.image is None: return
        frame = self.image.copy()
        for det in self.detection_results:
            bbox = det.get("bbox")
            label = det.get("unique_label","")
            obj_class = det.get("class","unknown")
            distance = det.get("distance", None)
            if bbox:
                x1,y1,x2,y2 = map(int,bbox)
                color = (255,0,0) if obj_class=="car" else (0,255,0)
                cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
                text = label + (f": {distance:.2f}m" if distance else "")
                cv2.putText(frame,text,(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)
        self.display_frame = frame

    def forward_to_onboard(self, app, frame):
        ros_img = self.cv2_to_ros_image(frame)
        ros_img.header = self.last_header
        #print("last_header onboard  ",ros_img.header)
        if app=="lane_detection":
            self.lane_img_pub.publish(ros_img)
        else:
            self.yolo_img_pub.publish(ros_img)

        #print(Color.YELLOW+f"{app}: Frame sent to onboard processing"+Color.RESET)

    def stop_timer(self, frame_id, source):
        if frame_id not in self.frame_ts:
            return

        t0 = self.frame_ts.pop(frame_id)
        e2e = (time.time() - t0) * 1000.0
        print(f"offload_e2e: {e2e:.2f} ms")

        rospy.loginfo(
            Color.GREEN +
            f"[E2E {source}] {frame_id} ? {e2e:.2f} ms" +
            Color.RESET
        )    

    def cv2_to_ros_image(self, cv_img):
        ros_img = Image()
        ros_img.height, ros_img.width = cv_img.shape[:2]
        ros_img.encoding = 'bgr8'
        ros_img.is_bigendian = 0
        ros_img.step = cv_img.shape[1]*3
        ros_img.data = cv_img.tobytes()
        return ros_img

    def display_loop(self):
        rate = rospy.Rate(30)
        while self.is_running and not rospy.is_shutdown():
            if self.display_frame is not None:
                cv2.imshow("Edge Detection View", self.display_frame)
                cv2.waitKey(1)
            rate.sleep()


if __name__=="__main__":
    LoadBalancerNode('load_balancer')

