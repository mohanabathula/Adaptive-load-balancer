#!/usr/bin/env python3
"""
Adaptive Load Balancer V5 - TRUE ALB WITH OPTIMIZATIONS + GPU TRACKING
=======================================================================
- Makes INDEPENDENT decisions for each application
- Optimizes onboard: YOLO runs once, results shared
- Optimizes edge: Parallel requests, not sequential
- Tracks per-application latency correctly
- NEW: GPU compute/idle time tracking for both onboard and edge scenarios

Author: Vidya Vepoori
Date: 2026-02-06
Version: 5.0 TRUE ALB + GPU TRACKING
"""

import os
import cv2
import signal
import rospy
import time
import csv
import json
import threading
import numpy as np
import requests
import psutil
import subprocess
import math
from concurrent.futures import ThreadPoolExecutor, as_completed
from collections import defaultdict
from pynput import keyboard

# ROS message types
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaCollisionEvent

# YOLO and PyTorch
from ultralytics import YOLO
import torch

# ============================================================================
# CONFIGURATION
# ============================================================================

CSV_FILE = "/media/root/b9f388cb-81d8-4121-93fb-ad514e324552/vidyav/main/alb_plot_gpu_tracking_1.csv"
YOLO_OBJECT_MODEL_PATH = "yolov5su.pt"  # For collision avoidance (cars, pedestrians)
YOLO_TRAFFIC_MODEL_PATH = "yolo11s.pt"   # For traffic signs and lights
NETWORK_INTERFACE = "enp25s0f0"

# Application requirements
APPLICATION_TABLE = [
    {"application": "collision_avoidance", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "traffic_light_detection", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "traffic_sign_detection", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "lane_detection", "latency_sensitivity": "high", "accuracy_priority": "high"},
]

# ALB Thresholds
RESOURCE_THRESHOLD_PERCENT =  12
BANDWIDTH_HIGH_THRESHOLD_MBPS = 10
BANDWIDTH_LOW_THRESHOLD_MBPS = 5

# Edge server
EDGE_SERVER_URL = "http://192.168.20.16:30052/processimage"
EDGE_TIMEOUT_SEC = 3.0
EDGE_CHECK_INTERVAL_SEC = 5.0

# Vehicle control
TARGET_SPEED_KMH = 30
SAFE_DISTANCE_M = 3.0
TARGET_VEHICLE_LABEL = "car_1"
MAX_THROTTLE = 0.8

# Image processing
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
YOLO_CONF_THRESHOLD = 0.4
ARTIFICIAL_DELAY_MS = 0

# Active applications
ACTIVE_APPLICATIONS = ['collision_avoidance', 'lane_detection'] #, 'traffic_light_detection', 'traffic_sign_detection',

# YOLO class mappings
TRAFFIC_SIGN_CLASSES = ['stop sign', 'traffic sign']
TRAFFIC_LIGHT_CLASSES = ['traffic light']
VEHICLE_CLASSES = ['car', 'truck', 'bus']
PERSON_CLASSES = ['person']

# Colors
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
# BACKGROUND RESOURCE MONITOR
# ============================================================================

class BackgroundResourceMonitor:
    """Background thread that continuously monitors resources."""
    
    def __init__(self, interface=NETWORK_INTERFACE, update_rate=10):
        self.interface = interface
        self.update_interval = 1.0 / update_rate
        self.lock = threading.Lock()
        self.cpu_percent = 0.0
        self.gpu_percent = 0.0
        self.ram_percent = 0.0
        self.tx_mbps = 0.0
        self.rx_mbps = 0.0
        self.last_net_counters = None
        self.last_net_time = time.time()
        self.running = True
        self.thread = None
    
    def start(self):
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
        print(f"{Color.GREEN}[MONITOR] Background resource monitor started{Color.RESET}")
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def get_resources(self):
        with self.lock:
            return {
                'cpu_percent': self.cpu_percent,
                'gpu_percent': self.gpu_percent,
                'ram_percent': self.ram_percent,
                'tx_mbps': self.tx_mbps,
                'rx_mbps': self.rx_mbps
            }
    
    def _monitor_loop(self):
        while self.running:
            try:
                loop_start = time.time()
                cpu = psutil.cpu_percent(interval=0)
                ram = psutil.virtual_memory().percent
                gpu = self._get_gpu_usage()
                tx, rx = self._get_network_bandwidth()
                
                with self.lock:
                    self.cpu_percent = cpu
                    self.gpu_percent = gpu
                    self.ram_percent = ram
                    self.tx_mbps = tx
                    self.rx_mbps = rx
                
                elapsed = time.time() - loop_start
                sleep_time = max(0, self.update_interval - elapsed)
                time.sleep(sleep_time)
            except Exception as e:
                time.sleep(0.1)
    
    def _get_gpu_usage(self):
        try:
            result = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
                universal_newlines=True
            )
            return int(result.strip().split("\n")[0])
        except:
            return 0
    
    def _get_network_bandwidth(self):
        try:
            net = psutil.net_io_counters(pernic=True)
            if self.interface not in net:
                return 0.0, 0.0
            
            current = net[self.interface]
            now = time.time()
            
            if self.last_net_counters is None:
                self.last_net_counters = current
                self.last_net_time = now
                return 0.0, 0.0
            
            interval = now - self.last_net_time
            if interval <= 0:
                return 0.0, 0.0
            
            tx_mbps = (current.bytes_sent - self.last_net_counters.bytes_sent) * 8 / (interval * 1024 * 1024)
            rx_mbps = (current.bytes_recv - self.last_net_counters.bytes_recv) * 8 / (interval * 1024 * 1024)
            
            self.last_net_counters = current
            self.last_net_time = now
            
            return round(tx_mbps, 2), round(rx_mbps, 2)
        except:
            return 0.0, 0.0

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_application_requirements(app_name):
    for app in APPLICATION_TABLE:
        if app["application"] == app_name:
            return {
                "latency_sensitivity": app["latency_sensitivity"],
                "accuracy_priority": app["accuracy_priority"]
            }
    return None

def get_speed_from_odom(odom_msg):
    if odom_msg is None:
        return 0.0
    vel = odom_msg.twist.twist.linear
    vx = vel.x
    vy = vel.y
    return math.sqrt(vx*vx + vy*vy) * 3.6

# ============================================================================
# TRUE ADAPTIVE LOAD BALANCER V5 + GPU TRACKING
# ============================================================================

class TrueAdaptiveLoadBalancerV5:
    """
    TRUE ALB V5: Makes independent decisions per application + GPU tracking.
    GPU Compute Time:
      - Onboard: Actual CUDA kernel execution time
      - Edge: Minimal (only image encoding)
    GPU Idle Time:
      - Onboard: Overhead (data transfer, etc.)
      - Edge: Network wait time
    """
    
    def __init__(self):
        print(f"\n{Color.BOLD}{Color.CYAN}{'='*70}")
        print(f"  TRUE ADAPTIVE LOAD BALANCER V5 + GPU TRACKING")
        print(f"  Independent decisions per app + Optimized execution")
        print(f"  GPU Compute/Idle Time Measurement")
        print(f"{'='*70}{Color.RESET}\n")
        
        rospy.init_node('true_alb_v5', anonymous=True)
        
        self.is_running = True
        self.frame_id = 0
        
        # Image data
        self.rgb_image = None
        self.depth_image = None
        self.display_frame = None
        
        # Cached YOLO results (for onboard optimization)
        self.cached_object_results = None  # yolov5su results
        self.cached_traffic_results = None  # yolo11s results
        self.cached_frame_id = -1
        
        # Detection results
        self.detection_results = []
        self.target_distance = None
        self.min_distance = None
        
        # Vehicle state
        self.ego_odom = None
        self.ego_moving = False
        self.collision_detected = False
        self.collision_actor_id = None
        
        self.lock = threading.Lock()
        
        # Edge server
        self.edge_available = True
        self.last_edge_check = 0
        
        # Thread pool for parallel edge requests
        self.edge_executor = ThreadPoolExecutor(max_workers=4)
        
        # ===== BACKGROUND RESOURCE MONITOR =====
        print(f"{Color.YELLOW}[INIT] Starting background resource monitor...{Color.RESET}")
        self.resource_monitor = BackgroundResourceMonitor(NETWORK_INTERFACE, 10)
        self.resource_monitor.start()
        time.sleep(0.5)
        print(f"{Color.GREEN}[INIT] ? Resource monitor active{Color.RESET}\n")
        
        # ===== YOLO Models =====
        print(f"{Color.YELLOW}[INIT] Loading YOLO models...{Color.RESET}")
        
        # Load object detection model (yolov5su)
        print(f"{Color.YELLOW}[INIT]   Loading {YOLO_OBJECT_MODEL_PATH} (objects)...{Color.RESET}")
        t_load_start = time.time()
        self.yolo_object_model = YOLO(YOLO_OBJECT_MODEL_PATH)
        if torch.cuda.is_available():
            self.yolo_object_model.to('cuda')
        t_obj_time = (time.time() - t_load_start) * 1000
        print(f"{Color.GREEN}[INIT]   ? Object model loaded: {t_obj_time:.1f}ms{Color.RESET}")
        
        # Load traffic detection model (yolo11s)
        print(f"{Color.YELLOW}[INIT]   Loading {YOLO_TRAFFIC_MODEL_PATH} (traffic)...{Color.RESET}")
        t_load_start = time.time()
        self.yolo_traffic_model = YOLO(YOLO_TRAFFIC_MODEL_PATH)
        if torch.cuda.is_available():
            self.yolo_traffic_model.to('cuda')
        t_traffic_time = (time.time() - t_load_start) * 1000
        print(f"{Color.GREEN}[INIT]   ? Traffic model loaded: {t_traffic_time:.1f}ms{Color.RESET}")
        
        if torch.cuda.is_available():
            print(f"{Color.GREEN}[INIT] ? Both models on GPU: {torch.cuda.get_device_name(0)}{Color.RESET}\n")
        else:
            print(f"{Color.YELLOW}[INIT] ? Both models on CPU{Color.RESET}\n")
        
        # ===== ROS Publishers =====
        self.control_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',
                                          CarlaEgoVehicleControl, queue_size=1)
        self.detection_pub = rospy.Publisher('/alb/detections', String, queue_size=10)
        
        # ===== ROS Subscribers =====
        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, 
                        self.image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/carla/ego_vehicle/depth_front/image', Image,
                        self.depth_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/carla/ego_vehicle/collision', CarlaCollisionEvent, self.collision_callback)
        
        # ===== CSV Logging with GPU tracking =====
        print(f"{Color.YELLOW}[INIT] CSV: {CSV_FILE}{Color.RESET}")
        self.csv_file = open(CSV_FILE, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "frame_id", "application_type", "processing_location",
            "cpu", "gpu", "ram", "tx_mbps", "rx_mbps",
            "edge_available", "app_inference_ms", "gpu_compute_ms", "gpu_idle_ms",
            "resource_check_ms", "decision_ms", "true_e2e_latency_ms", 
            "vehicle_speed_kmh", "target_distance_m", "min_distance_m", 
            "num_detections", "collision_detected", "collision_actor_id"
        ])
        self.csv_file.flush()
        print(f"{Color.GREEN}[INIT] ? CSV initialized with GPU tracking{Color.RESET}\n")
        
        # ===== Signal Handlers =====
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)
        
        # ===== Background Threads =====
        print(f"{Color.YELLOW}[INIT] Starting threads...{Color.RESET}")
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        self.keyboard_thread = threading.Thread(target=self.keyboard_control, daemon=True)
        self.keyboard_thread.start()
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        print(f"{Color.GREEN}[INIT] ? All threads started{Color.RESET}\n")
        
        print(f"{Color.BOLD}{Color.GREEN}{'='*70}")
        print(f"  SYSTEM READY")
        print(f"{'='*70}{Color.RESET}\n")
        print(f"{Color.CYAN}Controls: 's' = START | 'a' = STOP{Color.RESET}\n")
    
    def shutdown(self, signum=None, frame=None):
        print(f"\n{Color.RED}[SHUTDOWN] Stopping ALB V5...{Color.RESET}")
        self.is_running = False
        
        if hasattr(self, 'edge_executor'):
            self.edge_executor.shutdown(wait=False)
        if hasattr(self, 'resource_monitor'):
            self.resource_monitor.stop()
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        if hasattr(self, 'control_pub'):
            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            self.control_pub.publish(control_msg)
        
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown")
        print(f"{Color.GREEN}[SHUTDOWN] ? Complete{Color.RESET}\n")
    
    # ========================================================================
    # ROS CALLBACKS
    # ========================================================================
    
    def image_callback(self, ros_image):
        try:
            frame_rx_time = time.time()
            current_frame_id = self.frame_id
            
            print(f"\n{Color.CYAN}{'?'*70}")
            print(f"[FRAME {current_frame_id}] Received")
            print(f"{'?'*70}{Color.RESET}")
            
            if ARTIFICIAL_DELAY_MS > 0:
                time.sleep(ARTIFICIAL_DELAY_MS / 1000.0)
            
            if ros_image.encoding in ['bgra8', 'bgr8']:
                img_np = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                    (ros_image.height, ros_image.width, -1))
                frame = img_np[:, :, :3]
            else:
                return
            
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            
            with self.lock:
                self.rgb_image = frame
            
            self.handle_frame(frame, current_frame_id, frame_rx_time)
            self.frame_id += 1
            
        except Exception as e:
            rospy.logerr(f"[IMAGE] Error: {e}")
            import traceback
            traceback.print_exc()
    
    def depth_callback(self, ros_image):
        try:
            depth_np = np.frombuffer(ros_image.data, dtype=np.float32).reshape(
                (ros_image.height, ros_image.width))
            with self.lock:
                self.depth_image = depth_np
        except:
            pass
    
    def odometry_callback(self, msg):
        self.ego_odom = msg
    
    def collision_callback(self, msg):
        if hasattr(msg, 'other_actor_id') and msg.other_actor_id != 0:
            self.collision_detected = True
            self.collision_actor_id = msg.other_actor_id
            print(f"{Color.RED}[COLLISION] Actor {msg.other_actor_id}!{Color.RESET}")
        else:
            self.collision_detected = False
            self.collision_actor_id = None
    
    # ========================================================================
    # TRUE ALB CORE - PER-APP DECISIONS + GPU TRACKING
    # ========================================================================
    
    def handle_frame(self, frame, frame_id, frame_rx_time):
        """
        TRUE ALB with GPU tracking for both onboard and edge scenarios.
        """
        
        try:
            # ============================================================
            # STEP 1: Get system resources (once per frame)
            # ============================================================
            t_res_start = time.time()
            resources = self.resource_monitor.get_resources()
            edge_available = self.check_edge_server_available()
            t_res_end = time.time()
            resource_check_ms = (t_res_end - t_res_start) * 1000.0
            
            print(f"\n{Color.BLUE}[RESOURCES] CPU: {resources['cpu_percent']:.1f}% | "
                  f"GPU: {resources['gpu_percent']:.1f}% | RAM: {resources['ram_percent']:.1f}% | "
                  f"TX: {resources['tx_mbps']:.2f} Mbps | Edge: {'?' if edge_available else '?'}{Color.RESET}")
            
            # ============================================================
            # STEP 2: Make INDEPENDENT decision for EACH application
            # ============================================================
            app_decisions = {}
            total_decision_ms = 0.0
            
            for app in ACTIVE_APPLICATIONS:
                requirements = get_application_requirements(app)
                if not requirements:
                    continue
                
                t_dec_start = time.time()
                location = self.decide_processing_location(
                    requirements["latency_sensitivity"],
                    requirements["accuracy_priority"],
                    resources['cpu_percent'],
                    resources['ram_percent'],
                    resources['gpu_percent'],
                    resources['tx_mbps'],
                    resources['rx_mbps'],
                    edge_available
                )
                t_dec_end = time.time()
                decision_ms = (t_dec_end - t_dec_start) * 1000.0
                total_decision_ms += decision_ms
                
                app_decisions[app] = location
                
                print(f"{Color.YELLOW}[ALB] {app:30s} ? {location.upper():15s}{Color.RESET}")
            
            # ============================================================
            # STEP 3: Group apps by location for optimized execution
            # ============================================================
            onboard_apps = [app for app, loc in app_decisions.items() if loc == "onboard"]
            edge_apps = [app for app, loc in app_decisions.items() if loc == "edge"]
            edge_lowres_apps = [app for app, loc in app_decisions.items() if loc == "edge_lowres"]
            
            print(f"\n{Color.MAGENTA}[EXECUTION PLAN]{Color.RESET}")
            print(f"  Onboard ({len(onboard_apps)}): {onboard_apps}")
            print(f"  Edge ({len(edge_apps)}): {edge_apps}")
            print(f"  Edge LowRes ({len(edge_lowres_apps)}): {edge_lowres_apps}")
            
            # ============================================================
            # STEP 4: Execute onboard apps with GPU tracking
            # ============================================================
            onboard_inference_ms = 0.0
            onboard_gpu_compute_ms = 0.0
            onboard_gpu_idle_ms = 0.0
            
            if onboard_apps:
                print(f"\n{Color.GREEN}[ONBOARD] Processing {len(onboard_apps)} apps with GPU tracking...{Color.RESET}")
                onboard_inference_ms, onboard_gpu_compute_ms, onboard_gpu_idle_ms = \
                    self.process_onboard_batch_with_gpu_tracking(frame, frame_id, onboard_apps)
            
            # ============================================================
            # STEP 5: Execute edge apps with GPU tracking (PARALLEL)
            # ============================================================
            edge_results = {}
            
            if edge_apps:
                print(f"\n{Color.CYAN}[EDGE] Sending {len(edge_apps)} apps in parallel...{Color.RESET}")
                edge_results.update(
                    self.process_edge_batch_parallel_with_gpu_tracking(frame, frame_id, edge_apps)
                )
            
            if edge_lowres_apps:
                lowres = cv2.resize(frame, (320, 240))
                print(f"\n{Color.CYAN}[EDGE LOWRES] Sending {len(edge_lowres_apps)} apps...{Color.RESET}")
                edge_results.update(
                    self.process_edge_batch_parallel_with_gpu_tracking(lowres, frame_id, edge_lowres_apps)
                )
            
            # ============================================================
            # STEP 6: Calculate timing and log per-app with GPU tracking
            # ============================================================
            t_complete = time.time()
            total_frame_ms = (t_complete - frame_rx_time) * 1000.0
            
            speed_kmh = get_speed_from_odom(self.ego_odom)
            num_detections = len(self.detection_results)
            
            # Print summary with GPU breakdown
            print(f"\n{Color.BOLD}{Color.GREEN}?????????????????????????????????????????????")
            print(f"?  TRUE ALB V5 + GPU TRACKING - Frame {frame_id}")
            print(f"?????????????????????????????????????????????")
            print(f"?  Resource Check:         {resource_check_ms:>8.2f} ms")
            print(f"?  Decision (all apps):    {total_decision_ms:>8.2f} ms")
            
            if onboard_apps:
                print(f"?  ")
                print(f"?  Onboard ({len(onboard_apps)} apps):")
                print(f"?    Inference:      {onboard_inference_ms:>8.2f} ms")
                print(f"?    ?? GPU Compute: {onboard_gpu_compute_ms:>8.2f} ms  ?")
                print(f"?    ?? GPU Idle:    {onboard_gpu_idle_ms:>8.2f} ms  ?")
            
            if edge_results:
                print(f"?  ")
                print(f"?  Edge ({len(edge_results)} apps):")
                for app, metrics in edge_results.items():
                    print(f"?    {app[:20]:20s}")
                    print(f"?      E2E:          {metrics['inference_ms']:>8.2f} ms")
                    print(f"?      ?? GPU Comp:  {metrics['gpu_compute_ms']:>8.2f} ms  ?")
                    print(f"?      ?? GPU Idle:  {metrics['gpu_idle_ms']:>8.2f} ms  ?")
            
            print(f"?  ?????????????????????????????????????????")
            print(f"?  TOTAL FRAME TIME:       {total_frame_ms:>8.2f} ms  ?")
            print(f"?????????????????????????????????????????????")
            print(f"?  Detections:   {num_detections}")
            print(f"?  Target Dist:  {self.target_distance if self.target_distance else 'N/A'}")
            print(f"?  Speed:        {speed_kmh:.1f} km/h")
            print(f"?????????????????????????????????????????????{Color.RESET}")
            
            # ============================================================
            # STEP 7: Log to CSV (one row per app) with GPU metrics
            # ============================================================
            for app in ACTIVE_APPLICATIONS:
                location = app_decisions.get(app, "unknown")
                
                if location == "onboard":
                    app_inference_ms = onboard_inference_ms / max(1, len(onboard_apps))
                    gpu_compute_ms = onboard_gpu_compute_ms / max(1, len(onboard_apps))
                    gpu_idle_ms = onboard_gpu_idle_ms / max(1, len(onboard_apps))
                elif app in edge_results:
                    app_inference_ms = edge_results[app]['inference_ms']
                    gpu_compute_ms = edge_results[app]['gpu_compute_ms']
                    gpu_idle_ms = edge_results[app]['gpu_idle_ms']
                else:
                    app_inference_ms = 0.0
                    gpu_compute_ms = 0.0
                    gpu_idle_ms = 0.0
                
                self.log_to_csv(
                    timestamp=time.time(),
                    frame_id=frame_id,
                    app_type=app,
                    location=location,
                    cpu=resources['cpu_percent'],
                    gpu=resources['gpu_percent'],
                    ram=resources['ram_percent'],
                    tx_mbps=resources['tx_mbps'],
                    rx_mbps=resources['rx_mbps'],
                    edge_available=1 if edge_available else 0,
                    app_inference_ms=app_inference_ms,
                    gpu_compute_ms=gpu_compute_ms,
                    gpu_idle_ms=gpu_idle_ms,
                    resource_check_ms=resource_check_ms / len(ACTIVE_APPLICATIONS),
                    decision_ms=total_decision_ms / len(ACTIVE_APPLICATIONS),
                    total_frame_ms=total_frame_ms,
                    speed_kmh=speed_kmh,
                    target_dist=self.target_distance,
                    min_dist=self.min_distance,
                    num_detections=num_detections,
                    collision=1 if self.collision_detected else 0,
                    collision_id=self.collision_actor_id
                )
            
        except Exception as e:
            rospy.logerr(f"[ALB] Error: {e}")
            import traceback
            traceback.print_exc()
    
    def decide_processing_location(self, latency_sensitivity, accuracy_priority,
                                   cpu, ram, gpu, tx_mbps, rx_mbps, edge_available):
        """ALB decision logic per application."""
        
        onboard_ok = (cpu < RESOURCE_THRESHOLD_PERCENT and gpu < RESOURCE_THRESHOLD_PERCENT)
        bw_high = (tx_mbps >= BANDWIDTH_HIGH_THRESHOLD_MBPS)
        bw_low = (tx_mbps < BANDWIDTH_LOW_THRESHOLD_MBPS)
        bw_zero = (tx_mbps == 0)
        
        if not edge_available:
            return "onboard"
        
        if latency_sensitivity == "high":
            if onboard_ok:
                return "onboard"
            elif bw_high:
                return "edge"
            else:
                return "onboard"
        else:
            if accuracy_priority == "high":
                if bw_high:
                    return "edge"
                else:
                    return "onboard"
            else:
                if bw_zero:
                    return "onboard"
                elif bw_low:
                    return "edge_lowres"
                else:
                    return "edge"
    
    def check_edge_server_available(self):
        now = time.time()
        if now - self.last_edge_check < EDGE_CHECK_INTERVAL_SEC:
            return self.edge_available
        
        try:
            resp = requests.head(EDGE_SERVER_URL, timeout=2.0)
            self.edge_available = resp.status_code < 500
        except:
            self.edge_available = False
        
        self.last_edge_check = now
        return self.edge_available
    
    # ========================================================================
    # ONBOARD PROCESSING WITH GPU TRACKING
    # ========================================================================
    
    def process_onboard_batch_with_gpu_tracking(self, frame, frame_id, apps):
        """
        Process multiple apps onboard with GPU compute/idle tracking.
        Returns: (inference_ms, gpu_compute_ms, gpu_idle_ms)
        """
        
        try:
            total_inference_start = time.time()
            total_gpu_compute_ms = 0.0
            
            # Determine which models we need
            need_object_model = any(app == 'collision_avoidance' for app in apps)
            need_traffic_model = any(app in ['traffic_sign_detection', 'traffic_light_detection'] for app in apps)
            
            detections = []
            
            # ============================================================
            # Run Object Detection Model with GPU tracking
            # ============================================================
            if need_object_model:
                if torch.cuda.is_available():
                    torch.cuda.synchronize()
                    start_event = torch.cuda.Event(enable_timing=True)
                    end_event = torch.cuda.Event(enable_timing=True)
                    start_event.record()
                
                if self.cached_frame_id != frame_id:
                    print(f"{Color.GREEN}[ONBOARD] Running yolov5su with GPU tracking...{Color.RESET}")
                    results = self.yolo_object_model.predict(frame, conf=YOLO_CONF_THRESHOLD, verbose=False)[0]
                    self.cached_object_results = results
                else:
                    print(f"{Color.GREEN}[ONBOARD] Using cached yolov5su ?{Color.RESET}")
                    results = self.cached_object_results
                
                if torch.cuda.is_available():
                    end_event.record()
                    torch.cuda.synchronize()
                    obj_gpu_compute_ms = start_event.elapsed_time(end_event)
                    total_gpu_compute_ms += obj_gpu_compute_ms
                    print(f"{Color.BLUE}[GPU] yolov5su compute: {obj_gpu_compute_ms:.2f}ms{Color.RESET}")
                
                # Extract vehicle/pedestrian detections
                car_count = 0
                for r in results.boxes:
                    conf = float(r.conf[0])
                    cls_idx = int(r.cls[0])
                    label = results.names[cls_idx].lower()
                    bbox = r.xyxy[0].tolist()
                    
                    if any(v in label for v in VEHICLE_CLASSES) or label in PERSON_CLASSES:
                        car_count += 1
                        distance = self._calculate_distance(bbox)
                        detections.append({
                            'bbox': bbox,
                            'class': label,
                            'confidence': conf,
                            'unique_label': f"{label}_{car_count}",
                            'distance': distance
                        })
            
            # ============================================================
            # Run Traffic Detection Model with GPU tracking
            # ============================================================
            if need_traffic_model:
                if torch.cuda.is_available():
                    torch.cuda.synchronize()
                    start_event = torch.cuda.Event(enable_timing=True)
                    end_event = torch.cuda.Event(enable_timing=True)
                    start_event.record()
                
                if self.cached_frame_id != frame_id:
                    print(f"{Color.GREEN}[ONBOARD] Running yolo11s with GPU tracking...{Color.RESET}")
                    results = self.yolo_traffic_model.predict(frame, conf=YOLO_CONF_THRESHOLD, verbose=False)[0]
                    self.cached_traffic_results = results
                else:
                    print(f"{Color.GREEN}[ONBOARD] Using cached yolo11s ?{Color.RESET}")
                    results = self.cached_traffic_results
                
                if torch.cuda.is_available():
                    end_event.record()
                    torch.cuda.synchronize()
                    traffic_gpu_compute_ms = start_event.elapsed_time(end_event)
                    total_gpu_compute_ms += traffic_gpu_compute_ms
                    print(f"{Color.BLUE}[GPU] yolo11s compute: {traffic_gpu_compute_ms:.2f}ms{Color.RESET}")
            
            # Update cache
            if need_object_model or need_traffic_model:
                self.cached_frame_id = frame_id
            
            # Store detections
            with self.lock:
                self.detection_results = detections
                
                self.target_distance = None
                for det in detections:
                    if det['unique_label'] == TARGET_VEHICLE_LABEL:
                        self.target_distance = det['distance']
                        break
                
                distances = [d['distance'] for d in detections if d['distance'] is not None]
                self.min_distance = min(distances) if distances else None
            
            self.detection_pub.publish(String(data=json.dumps(detections)))
            self.update_display(frame, detections)
            
            # Calculate total times
            total_inference_ms = (time.time() - total_inference_start) * 1000.0
            gpu_idle_ms = total_inference_ms - total_gpu_compute_ms
            
            print(f"{Color.GREEN}[ONBOARD] Total: {total_inference_ms:.2f}ms | "
                  f"GPU Compute: {total_gpu_compute_ms:.2f}ms | GPU Idle: {gpu_idle_ms:.2f}ms{Color.RESET}")
            
            return total_inference_ms, total_gpu_compute_ms, gpu_idle_ms
            
        except Exception as e:
            rospy.logerr(f"[ONBOARD] Error: {e}")
            import traceback
            traceback.print_exc()
            return 0.0, 0.0, 0.0
    
    def _calculate_distance(self, bbox):
        if self.depth_image is None:
            return None
        try:
            x1, y1, x2, y2 = map(int, bbox)
            x1 = max(0, min(x1, self.depth_image.shape[1]-1))
            x2 = max(0, min(x2, self.depth_image.shape[1]-1))
            y1 = max(0, min(y1, self.depth_image.shape[0]-1))
            y2 = max(0, min(y2, self.depth_image.shape[0]-1))
            
            if x2 > x1 and y2 > y1:
                patch = self.depth_image[y1:y2, x1:x2]
                if patch.size > 0:
                    return float(np.median(patch))
        except:
            pass
        return None
    
    # ========================================================================
    # EDGE PROCESSING WITH GPU TRACKING (PARALLEL)
    # ========================================================================
    
    def process_edge_batch_parallel_with_gpu_tracking(self, frame, frame_id, apps):
        """
        Send multiple apps to edge in PARALLEL with GPU tracking.
        Returns dict: {app: {'inference_ms', 'gpu_compute_ms', 'gpu_idle_ms'}}
        """
        
        results = {}
        
        # Encode once and measure GPU time for encoding
        if torch.cuda.is_available():
            torch.cuda.synchronize()
            t_encode_start = time.time()
        
        success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        if torch.cuda.is_available():
            torch.cuda.synchronize()
            encode_gpu_compute_ms = (time.time() - t_encode_start) * 1000.0
        else:
            encode_gpu_compute_ms = 0.0
        
        if not success:
            return results
        
        # Submit all requests in parallel
        futures = {}
        for app in apps:
            future = self.edge_executor.submit(
                self._send_edge_request_with_gpu_tracking, 
                buffer.tobytes(), frame.shape, frame_id, app, frame, encode_gpu_compute_ms
            )
            futures[future] = app
        
        # Collect results
        for future in as_completed(futures):
            app = futures[future]
            try:
                metrics = future.result(timeout=EDGE_TIMEOUT_SEC)
                results[app] = metrics
                print(f"{Color.CYAN}[EDGE] {app:30s} ? E2E: {metrics['inference_ms']:.2f}ms "
                      f"(GPU Compute: {metrics['gpu_compute_ms']:.2f}ms, Idle: {metrics['gpu_idle_ms']:.2f}ms){Color.RESET}")
            except Exception as e:
                print(f"{Color.RED}[EDGE] {app} failed: {e}{Color.RESET}")
                results[app] = {'inference_ms': 0.0, 'gpu_compute_ms': 0.0, 'gpu_idle_ms': 0.0}
        
        return results
    
    def _send_edge_request_with_gpu_tracking(self, img_bytes, frame_shape, frame_id, app, frame, encode_gpu_ms):
        """
        Send single edge request with GPU tracking.
        GPU Compute = encoding time (local)
        GPU Idle = network wait time
        """
        
        headers = {
            'Content-Type': 'application/octet-stream',
            'Frame-Width': str(frame_shape[1]),
            'Frame-Height': str(frame_shape[0]),
            'Frame-Identifier': f"frame_{frame_id}",
            'detection-flag': app,
        }
        
        # Network wait starts (GPU goes idle)
        t_network_start = time.time()
        resp = requests.post(EDGE_SERVER_URL, data=img_bytes, headers=headers, timeout=EDGE_TIMEOUT_SEC)
        t_network_end = time.time()
        
        total_e2e_ms = (t_network_end - t_network_start) * 1000.0
        gpu_idle_ms = total_e2e_ms  # All network time is GPU idle
        
        if resp.status_code != 200:
            return {'inference_ms': 0.0, 'gpu_compute_ms': encode_gpu_ms, 'gpu_idle_ms': gpu_idle_ms}
        
        # Parse detections if collision_avoidance
        if app == 'collision_avoidance':
            try:
                result = resp.json()
                if 'OBD_model' in result:
                    self._parse_collision_detections(result, frame)
            except:
                pass
        
        return {
            'inference_ms': total_e2e_ms,
            'gpu_compute_ms': encode_gpu_ms,
            'gpu_idle_ms': gpu_idle_ms
        }
    
    def _parse_collision_detections(self, result, frame):
        """Parse collision detection results from edge."""
        
        try:
            obd_data = result['OBD_model']
            if isinstance(obd_data, str):
                obd_data = json.loads(obd_data)
            
            raw_detections = obd_data.get('detections', [])
            detections = []
            car_count = 0
            
            for det in raw_detections:
                if det.get("class", "").lower() in ['car', 'truck', 'bus']:
                    car_count += 1
                    det['unique_label'] = f"{det.get('class')}_{car_count}"
                    det['distance'] = self._calculate_distance(det.get("bbox"))
                    detections.append(det)
            
            with self.lock:
                self.detection_results = detections
                
                self.target_distance = None
                for det in detections:
                    if det.get('unique_label') == TARGET_VEHICLE_LABEL:
                        self.target_distance = det.get('distance')
                        break
                
                distances = [d.get('distance') for d in detections if d.get('distance') is not None]
                self.min_distance = min(distances) if distances else None
            
            self.detection_pub.publish(String(data=json.dumps(detections)))
            self.update_display(frame, detections)
            
        except Exception as e:
            pass
    
    # ========================================================================
    # VEHICLE CONTROL
    # ========================================================================
    
    def control_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.is_running:
            try:
                self.control_vehicle()
            except:
                pass
            rate.sleep()
    
    def control_vehicle(self):
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = 0.0
        control_msg.hand_brake = False
        control_msg.reverse = False
        
        speed = get_speed_from_odom(self.ego_odom)
        
        if not self.ego_moving:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
        elif self.target_distance and self.target_distance <= SAFE_DISTANCE_M:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
        elif speed < TARGET_SPEED_KMH:
            control_msg.throttle = MAX_THROTTLE
            control_msg.brake = 0.0
        else:
            control_msg.throttle = 0.0
            control_msg.brake = 0.0
        
        self.control_pub.publish(control_msg)
    
    def keyboard_control(self):
        def on_press(key):
            try:
                if hasattr(key, 'char'):
                    if key.char == 's':
                        self.ego_moving = True
                        print(f"\n{Color.GREEN}[KEY] START{Color.RESET}\n")
                    elif key.char == 'a':
                        self.ego_moving = False
                        print(f"\n{Color.RED}[KEY] STOP{Color.RESET}\n")
            except:
                pass
        
        from pynput import keyboard
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        listener.join()
    
    # ========================================================================
    # DISPLAY
    # ========================================================================
    
    def update_display(self, frame, detections):
        display = frame.copy()
        for det in detections:
            bbox = det.get("bbox")
            if not bbox:
                continue
            x1, y1, x2, y2 = map(int, bbox)
            dist = det.get("distance")
            color = (0, 0, 255) if (dist and dist <= SAFE_DISTANCE_M) else (0, 255, 0)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            text = det.get("unique_label", "")
            if dist:
                text += f" {dist:.2f}m"
            cv2.putText(display, text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        with self.lock:
            self.display_frame = display
    
    def display_loop(self):
        rate = rospy.Rate(30)
        while self.is_running and not rospy.is_shutdown():
            if self.display_frame is not None:
                cv2.imshow("ALB V5 + GPU Tracking", self.display_frame)
                cv2.waitKey(1)
            rate.sleep()
    
    # ========================================================================
    # CSV LOGGING WITH GPU METRICS
    # ========================================================================
    
    def log_to_csv(self, timestamp, frame_id, app_type, location, cpu, gpu, ram,
                   tx_mbps, rx_mbps, edge_available, app_inference_ms, gpu_compute_ms,
                   gpu_idle_ms, resource_check_ms, decision_ms, total_frame_ms, 
                   speed_kmh, target_dist, min_dist, num_detections, collision, collision_id):
        
        self.csv_writer.writerow([
            timestamp, frame_id, app_type, location, cpu, gpu, ram, tx_mbps, rx_mbps,
            edge_available, app_inference_ms, gpu_compute_ms, gpu_idle_ms,
            resource_check_ms, decision_ms, total_frame_ms, speed_kmh,
            target_dist if target_dist is not None else "",
            min_dist if min_dist is not None else "",
            num_detections, collision,
            collision_id if collision_id is not None else ""
        ])
        self.csv_file.flush()

# ============================================================================
# MAIN
# ============================================================================

def main():
    try:
        alb = TrueAdaptiveLoadBalancerV5()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"{Color.RED}[ERROR] {e}{Color.RESET}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
