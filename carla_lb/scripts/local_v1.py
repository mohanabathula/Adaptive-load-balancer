#!/usr/bin/env python3
"""
Complete Local Processing System with Vehicle Control & TRUE E2E Latency + GPU Tracking
========================================================================================
Added GPU compute/idle time tracking to separate productive GPU work from idle time.

Author: Vidya Vepoori
Date: 2026-02-05
Version: FINAL SIMPLIFIED - LOCAL ONLY + GPU TRACKING
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaCollisionEvent
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import json
import torch
import time
import csv
import psutil
import subprocess
import math
from pynput import keyboard

# ============================================================================
# CONFIGURATION
# ============================================================================

CSV_FILE = "/media/root/b9f388cb-81d8-4121-93fb-ad514e324552/vidyav/main/local_plot_gpu_tracking_1.csv"
ARTIFICIAL_DELAY_MS = 0   # Artificial E2E increase for testing

# Vehicle control parameters
TARGET_SPEED_KMH = 20
SAFE_DISTANCE_M = 3.0
TARGET_VEHICLE_LABEL = "car_1"
MAX_THROTTLE = 0.8

# ANSI color codes
class Color:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_gpu_usage_percent():
    """Get GPU utilization percentage using nvidia-smi."""
    try:
        result = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
            universal_newlines=True
        )
        return int(result.strip().split("\n")[0])
    except Exception as e:
        return 0

def get_network_bandwidth(interface="enp25s0f0", interval=0.01):
    """Measure network bandwidth (TX/RX in Mbps)."""
    try:
        import psutil
        net1 = psutil.net_io_counters(pernic=True).get(interface)
        if net1 is None:
            return {"tx_mbps": 0.0, "rx_mbps": 0.0}
        
        time.sleep(interval)
        net2 = psutil.net_io_counters(pernic=True).get(interface)
        if net2 is None:
            return {"tx_mbps": 0.0, "rx_mbps": 0.0}
        
        tx_mbps = (net2.bytes_sent - net1.bytes_sent) * 8 / (interval * 1024 * 1024)
        rx_mbps = (net2.bytes_recv - net1.bytes_recv) * 8 / (interval * 1024 * 1024)
        
        return {
            "tx_mbps": round(tx_mbps, 2),
            "rx_mbps": round(rx_mbps, 2)
        }
    except:
        return {"tx_mbps": 0.0, "rx_mbps": 0.0}

def get_speed_from_odom(odom_msg):
    """Calculate speed in km/h from odometry message."""
    if odom_msg is None:
        return 0.0
    vel = odom_msg.twist.twist.linear
    vx = vel.x
    vy = vel.y
    return math.sqrt(vx*vx + vy*vy) * 3.6  # Convert m/s to km/h

# ============================================================================
# MAIN LOCAL PROCESSING NODE WITH VEHICLE CONTROL + GPU TRACKING
# ============================================================================

class LocalProcessingWithControl:
    """
    Pure local (onboard) processing system with vehicle control + GPU tracking.
    TRUE E2E latency = Time from frame receipt to completion.
    GPU Compute Time = Actual CUDA kernel execution time
    GPU Idle Time = Time GPU is waiting (network, data transfer, etc.)
    """
    
    def __init__(self):
        rospy.init_node('local_processing_with_control', anonymous=True)
        
        print(f"\n{Color.BOLD}{Color.CYAN}{'='*80}")
        print(f"  LOCAL PROCESSING SYSTEM WITH VEHICLE CONTROL + GPU TRACKING")
        print(f"  PURE LOCAL - TRUE E2E LATENCY + GPU COMPUTE/IDLE MEASUREMENT")
        print(f"{'='*80}{Color.RESET}\n")

        # -------- ROS PARAMETERS --------
        self.rgb_topic = rospy.get_param('~rgb_topic', '/carla/ego_vehicle/rgb_front/image')
        self.depth_topic = rospy.get_param('~depth_topic', '/carla/ego_vehicle/depth_front/image')
        self.model_path = rospy.get_param('~model_path', 'yolov5su.pt')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.4)
        self.resized_width = rospy.get_param('~resized_width', 640)
        self.resized_height = rospy.get_param('~resized_height', 480)

        # -------- STATE VARIABLES --------
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame_id = 0
        
        # Detection results
        self.detection_results = []
        self.depth_image = None
        self.target_distance = None
        self.min_distance = None
        
        # Vehicle state
        self.ego_odom = None
        self.ego_moving = False
        self.collision_detected = False
        self.collision_actor_id = None

        # -------- LOAD YOLO MODEL --------
        print(f"{Color.YELLOW}[INIT] Loading YOLO model: {self.model_path}{Color.RESET}")
        t_load_start = time.time()
        
        self.model = YOLO(self.model_path)
        
        if torch.cuda.is_available():
            self.model.to('cuda')
            print(f"{Color.GREEN}[INIT] ? YOLO loaded on GPU: {torch.cuda.get_device_name(0)}{Color.RESET}")
        else:
            print(f"{Color.YELLOW}[INIT] ? YOLO loaded on CPU{Color.RESET}")
        
        t_load_end = time.time()
        print(f"{Color.GREEN}[INIT] ? Model loading time: {(t_load_end - t_load_start)*1000:.1f}ms{Color.RESET}\n")

        # -------- ROS SUBSCRIBERS --------
        self.rgb_sub = rospy.Subscriber(
            self.rgb_topic, Image, self.rgb_callback, queue_size=10
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            '/carla/ego_vehicle/odometry', Odometry, self.odometry_callback
        )
        self.collision_sub = rospy.Subscriber(
            '/carla/ego_vehicle/collision', CarlaCollisionEvent, self.collision_callback
        )

        # -------- ROS PUBLISHERS --------
        self.car_pub = rospy.Publisher('/carla/local_detection', String, queue_size=1)
        self.control_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd',
            CarlaEgoVehicleControl,
            queue_size=1
        )

        # -------- CSV LOGGING SETUP --------
        print(f"{Color.YELLOW}[INIT] Setting up CSV logger: {CSV_FILE}{Color.RESET}")
        self.csv_file = open(CSV_FILE, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write CSV header with GPU compute/idle fields
        self.csv_writer.writerow([
            "timestamp",
            "frame_id",
            "app_type",
            "location",
            "cpu",
            "gpu",
            "ram",
            "tx_mbps",
            "rx_mbps",
            "edge_available",
            "inference_ms",
            "gpu_compute_ms",
            "gpu_idle_ms",
            "resource_check_ms",
            "decision_ms",
            "overhead_ms",
            "true_e2e_ms",
            "speed_kmh",
            "target_dist",
            "min_dist",
            "num_detections",
            "collision",
            "collision_id"
        ])
        self.csv_file.flush()
        print(f"{Color.GREEN}[INIT] ? CSV logger initialized with GPU tracking{Color.RESET}\n")

        # -------- START BACKGROUND THREADS --------
        print(f"{Color.YELLOW}[INIT] Starting background threads...{Color.RESET}")
        
        # Control loop thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        # Keyboard control thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_control, daemon=True)
        self.keyboard_thread.start()
        
        print(f"{Color.GREEN}[INIT] ? All threads started{Color.RESET}\n")
        
        print(f"{Color.BOLD}{Color.GREEN}{'='*80}")
        print(f"  SYSTEM READY - Waiting for images...")
        print(f"{'='*80}{Color.RESET}\n")
        
        print(f"{Color.CYAN}Keyboard Controls:{Color.RESET}")
        print(f"  Press 's' to START vehicle")
        print(f"  Press 'a' to STOP vehicle (brake)")
        print(f"  Press Ctrl+C to shutdown\n")

        rospy.loginfo("Local processing node with GPU tracking initialized")

    # ========================================================================
    # ROS CALLBACKS
    # ========================================================================

    def rgb_callback(self, msg):
        """
        Main RGB image callback with TRUE E2E latency + GPU compute/idle tracking.
        """
        # ============================================================
        # T0: FRAME RECEIVE TIMESTAMP - START OF E2E MEASUREMENT
        # ============================================================
        t0_frame_recv = time.time()
        
        print(f"\n{Color.CYAN}{'?'*70}")
        print(f"[FRAME {self.frame_id}] Received at T0={t0_frame_recv:.6f}")
        print(f"{'?'*70}{Color.RESET}")

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (self.resized_width, self.resized_height))

            # ============================================================
            # ARTIFICIAL DELAY (if configured for testing)
            # ============================================================
            if ARTIFICIAL_DELAY_MS > 0:
                time.sleep(ARTIFICIAL_DELAY_MS / 1000.0)
                print(f"  ?  Applied artificial delay: {ARTIFICIAL_DELAY_MS}ms")

            # ============================================================
            # YOLO INFERENCE WITH GPU COMPUTE TIME TRACKING
            # ============================================================
            print(f"{Color.GREEN}[LOCAL] Processing frame {self.frame_id} locally with GPU tracking...{Color.RESET}")
            
            # Synchronize GPU before starting
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            
            # Track actual GPU compute time using CUDA events
            t_infer_start = time.time()
            
            if torch.cuda.is_available():
                start_event = torch.cuda.Event(enable_timing=True)
                end_event = torch.cuda.Event(enable_timing=True)
                start_event.record()
            
            # Run inference
            results = self.model.predict(cv_image, conf=self.conf_threshold, verbose=False)[0]
            
            if torch.cuda.is_available():
                end_event.record()
                torch.cuda.synchronize()
                gpu_compute_ms = start_event.elapsed_time(end_event)  # Actual GPU kernel time
            else:
                gpu_compute_ms = 0.0
            
            t_infer_end = time.time()
            inference_ms = (t_infer_end - t_infer_start) * 1000.0
            
            # Calculate GPU idle time (total inference time - actual GPU compute)
            gpu_idle_ms = inference_ms - gpu_compute_ms
            
            print(f"{Color.GREEN}[LOCAL] Inference time: {inference_ms:.2f}ms{Color.RESET}")
            print(f"{Color.BLUE}[GPU] Compute time: {gpu_compute_ms:.2f}ms | Idle time: {gpu_idle_ms:.2f}ms{Color.RESET}")

            # ============================================================
            # PARSE DETECTIONS AND CALCULATE DISTANCES
            # ============================================================
            with self.lock:
                self.detection_results = []
                car_count = 0

                for r in results.boxes:
                    conf = float(r.conf[0])
                    cls_idx = int(r.cls[0])
                    label = results.names[cls_idx].lower()
                    
                    if label != 'car':
                        continue

                    bbox = r.xyxy[0].tolist()
                    car_count += 1
                    unique_label = f"car_{car_count}"

                    # Calculate distance from depth image
                    distance = None
                    if self.depth_image is not None:
                        x1, y1, x2, y2 = map(int, bbox)
                        # Ensure bounds are valid
                        x1 = max(0, min(x1, self.depth_image.shape[1]-1))
                        x2 = max(0, min(x2, self.depth_image.shape[1]-1))
                        y1 = max(0, min(y1, self.depth_image.shape[0]-1))
                        y2 = max(0, min(y2, self.depth_image.shape[0]-1))
                        
                        if x2 > x1 and y2 > y1:
                            patch = self.depth_image[y1:y2, x1:x2]
                            if patch.size > 0:
                                distance = float(np.median(patch))

                    self.detection_results.append({
                        'bbox': bbox,
                        'class': 'car',
                        'confidence': conf,
                        'unique_label': unique_label,
                        'distance': distance
                    })

                # Find target distance (car_1) and min distance
                self.target_distance = None
                for det in self.detection_results:
                    if det['unique_label'] == TARGET_VEHICLE_LABEL:
                        self.target_distance = det['distance']
                        break
                
                # Find minimum distance
                distances = [d['distance'] for d in self.detection_results 
                           if d['distance'] is not None]
                self.min_distance = min(distances) if distances else None

            # ============================================================
            # DRAW BOUNDING BOXES AND DISPLAY
            # ============================================================
            display_image = cv_image.copy()
            for det in self.detection_results:
                x1, y1, x2, y2 = map(int, det['bbox'])
                distance = det['distance']
                
                # Color based on distance
                if distance is not None and distance <= SAFE_DISTANCE_M:
                    color = (0, 0, 255)  # Red - danger
                else:
                    color = (0, 255, 0)  # Green - safe
                
                cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 2)
                
                txt = det['unique_label']
                if distance is not None:
                    txt += f" {distance:.2f}m"
                
                cv2.putText(display_image, txt, (x1, y1-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow("Local Car Detection & Distance", display_image)
            cv2.waitKey(1)

            # ============================================================
            # PUBLISH DETECTIONS
            # ============================================================
            self.car_pub.publish(String(data=json.dumps(self.detection_results)))

            # ============================================================
            # T1: COMPLETE TIMESTAMP - END OF E2E MEASUREMENT
            # ============================================================
            t1_complete = time.time()
            
            # ============================================================
            # CALCULATE TRUE E2E LATENCY (T1 - T0)
            # ============================================================
            true_e2e_ms = (t1_complete - t0_frame_recv) * 1000.0
            
            # Calculate overhead (everything except inference)
            overhead_ms = true_e2e_ms - inference_ms

            # ============================================================
            # GET SYSTEM RESOURCES (for logging only, not part of E2E)
            # ============================================================
            cpu = psutil.cpu_percent(interval=0)
            ram = psutil.virtual_memory().percent
            gpu = get_gpu_usage_percent()
            bandwidth = get_network_bandwidth()
            tx_mbps = bandwidth['tx_mbps']
            rx_mbps = bandwidth['rx_mbps']

            # Get vehicle state
            speed_kmh = get_speed_from_odom(self.ego_odom)
            num_detections = len(self.detection_results)

            # ============================================================
            # PRINT TRUE E2E LATENCY BREAKDOWN WITH GPU TRACKING
            # ============================================================
            print(f"\n{Color.BOLD}{Color.GREEN}?????????????????????????????????????????????")
            print(f"?  TRUE E2E LATENCY - Frame {self.frame_id}")
            print(f"?????????????????????????????????????????????")
            print(f"?  Inference Time:     {inference_ms:>8.2f} ms")
            print(f"?    ?? GPU Compute:   {gpu_compute_ms:>8.2f} ms  ?")
            print(f"?    ?? GPU Idle:      {gpu_idle_ms:>8.2f} ms  ?")
            print(f"?  Overhead:           {overhead_ms:>8.2f} ms")
            print(f"?  ?????????????????????????????????????????")
            print(f"?  TRUE E2E LATENCY:   {true_e2e_ms:>8.2f} ms  ?")
            print(f"?????????????????????????????????????????????")
            print(f"?  Location:           ONBOARD (LOCAL)")
            print(f"?  Detections:         {num_detections} cars")
            print(f"?  Target Distance:    {self.target_distance if self.target_distance else 'N/A'}")
            print(f"?  Min Distance:       {self.min_distance if self.min_distance else 'N/A'}")
            print(f"?  Speed:              {speed_kmh:.1f} km/h")
            print(f"?  Collision:          {'YES' if self.collision_detected else 'NO'}")
            print(f"?????????????????????????????????????????????{Color.RESET}")

            # ============================================================
            # LOG TO CSV WITH GPU COMPUTE/IDLE FIELDS
            # ============================================================
            self.csv_writer.writerow([
                time.time(),                                          # timestamp
                self.frame_id,                                        # frame_id
                "collision_avoidance",                                # app_type
                "onboard",                                            # location
                cpu,                                                  # cpu
                gpu,                                                  # gpu
                ram,                                                  # ram
                tx_mbps,                                             # tx_mbps
                rx_mbps,                                             # rx_mbps
                0,                                                   # edge_available (always 0 for local)
                inference_ms,                                        # inference_ms
                gpu_compute_ms,                                      # gpu_compute_ms ?
                gpu_idle_ms,                                         # gpu_idle_ms ?
                0.0,                                                 # resource_check_ms (N/A for local)
                0.0,                                                 # decision_ms (N/A for local)
                overhead_ms,                                         # overhead_ms
                true_e2e_ms,                                        # true_e2e_ms ?
                speed_kmh,                                          # speed_kmh
                self.target_distance if self.target_distance is not None else "",  # target_dist
                self.min_distance if self.min_distance is not None else "",        # min_dist
                num_detections,                                     # num_detections
                1 if self.collision_detected else 0,               # collision
                self.collision_actor_id if self.collision_actor_id is not None else ""  # collision_id
            ])
            self.csv_file.flush()

            # Increment frame ID
            self.frame_id += 1

        except Exception as e:
            rospy.logwarn(f"RGB callback error: {e}")
            import traceback
            traceback.print_exc()

    def depth_callback(self, msg):
        """Depth image callback."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth_image.dtype != np.float32:
                depth_image = depth_image.astype(np.float32)
            with self.lock:
                self.depth_image = depth_image
        except Exception as e:
            rospy.logwarn(f"Depth callback error: {e}")

    def odometry_callback(self, msg):
        """Vehicle odometry callback."""
        self.ego_odom = msg

    def collision_callback(self, msg):
        """Collision event callback."""
        if hasattr(msg, 'other_actor_id') and msg.other_actor_id != 0:
            self.collision_detected = True
            self.collision_actor_id = msg.other_actor_id
            print(f"{Color.RED}{'!'*70}")
            print(f"[COLLISION] DETECTED! Actor ID: {msg.other_actor_id}")
            print(f"{'!'*70}{Color.RESET}")
        else:
            self.collision_detected = False
            self.collision_actor_id = None

    # ========================================================================
    # VEHICLE CONTROL
    # ========================================================================

    def control_loop(self):
        """Main vehicle control loop (runs at 20 Hz)."""
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            try:
                self.control_vehicle()
            except Exception as e:
                rospy.logerr(f"[CONTROL] Error: {e}")
            
            rate.sleep()

    def control_vehicle(self):
        """Execute vehicle control based on current state."""
        
        # Prepare control message
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = 0.0
        control_msg.hand_brake = False
        control_msg.reverse = False
        
        # Get current speed
        current_speed = get_speed_from_odom(self.ego_odom)
        
        # Format distance string
        dist_str = f"{self.target_distance:.2f}m" if self.target_distance else "N/A"
        
        # Control logic
        if not self.ego_moving:
            # Vehicle stopped by user
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            status = "STOPPED"
            color = Color.YELLOW
        
        elif self.target_distance is not None and self.target_distance <= SAFE_DISTANCE_M:
            # Emergency brake - too close to target
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            status = "EMERGENCY BRAKE"
            color = Color.RED
        
        elif current_speed < TARGET_SPEED_KMH:
            # Accelerate to target speed
            control_msg.throttle = MAX_THROTTLE
            control_msg.brake = 0.0
            status = "ACCELERATING"
            color = Color.GREEN
        
        else:
            # Cruising at target speed
            control_msg.throttle = 0.0
            control_msg.brake = 0.0
            status = "CRUISING"
            color = Color.CYAN
        
        # Publish control command
        self.control_pub.publish(control_msg)
        
        # Print status (throttled to avoid spam)
        if hasattr(self, '_last_control_print'):
            if time.time() - self._last_control_print > 0.5:  # Print every 500ms
                print(f"{color}[CONTROL] {status} | Speed: {current_speed:.1f} km/h | "
                      f"Distance: {dist_str} | Throttle: {control_msg.throttle:.2f} | "
                      f"Brake: {control_msg.brake:.2f}{Color.RESET}")
                self._last_control_print = time.time()
        else:
            self._last_control_print = time.time()

    def keyboard_control(self):
        """Handle keyboard input for vehicle control."""
        
        def on_press(key):
            try:
                if hasattr(key, 'char'):
                    if key.char == 's':
                        self.ego_moving = True
                        print(f"\n{Color.GREEN}{'='*70}")
                        print(f"  [KEYBOARD] ? START - Vehicle moving")
                        print(f"{'='*70}{Color.RESET}\n")
                    elif key.char == 'a':
                        self.ego_moving = False
                        print(f"\n{Color.RED}{'='*70}")
                        print(f"  [KEYBOARD] ? STOP - Applying brakes")
                        print(f"{'='*70}{Color.RESET}\n")
            except AttributeError:
                pass
        
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        listener.join()

    # ========================================================================
    # SHUTDOWN
    # ========================================================================

    def shutdown(self):
        """Clean shutdown handler."""
        print(f"\n{Color.RED}[SHUTDOWN] Shutting down Local Processing...{Color.RESET}")
        
        # Stop vehicle
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        self.control_pub.publish(control_msg)
        print(f"{Color.GREEN}[SHUTDOWN] ? Vehicle stopped{Color.RESET}")
        
        # Close resources
        self.csv_file.close()
        cv2.destroyAllWindows()
        
        print(f"{Color.GREEN}[SHUTDOWN] ? System shutdown complete{Color.RESET}\n")


# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    node = None
    try:
        node = LocalProcessingWithControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Local processing node terminated")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        if node is not None:
            node.shutdown()
        cv2.destroyAllWindows()
