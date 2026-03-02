#!/usr/bin/env python3
"""
Adaptive Load Balancer V5 - TRUE ALB WITH OPTIMIZATIONS + GPU TRACKING
=======================================================================
- Makes INDEPENDENT decisions for each application
- Optimizes onboard: YOLO runs once, results shared
- Optimizes edge: Parallel requests, not sequential
- Tracks per-application latency INDEPENDENTLY (not averaged)
- GPU compute/idle time tracking for both onboard and edge scenarios

FIXES vs V5 original:
  - Onboard apps get INDIVIDUAL timing per model used (not divided average)
  - Edge apps get INDIVIDUAL timing per request (not shared parallel time)
  - Each app's CSV row reflects its real latency contribution

Author: Vidya Vepoori
Date: 2026-02-06
Version: 1.1 TRUE ALB + INDEPENDENT PER-APP LATENCY
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

CSV_FILE = "/media/root/b9f388cb-81d8-4121-93fb-ad514e324552/vidyav/main/alb_plot.csv"
YOLO_OBJECT_MODEL_PATH = "yolov5su.pt"  # For collision avoidance (cars, pedestrians)
YOLO_TRAFFIC_MODEL_PATH = "yolo11s.pt"   # For traffic signs and lights
NETWORK_INTERFACE = "enp25s0f0"

# Application requirements
APPLICATION_TABLE = [
    {"application": "collision_avoidance",    "latency_sensitivity": "high", "accuracy_priority": "low"},
    {"application": "traffic_light_detection","latency_sensitivity": "low",  "accuracy_priority": "high"},
    {"application": "traffic_sign_detection", "latency_sensitivity": "low",  "accuracy_priority": "high"},
    {"application": "lane_detection",         "latency_sensitivity": "low",  "accuracy_priority": "high"},
]

# Which YOLO model each app uses onboard
# collision_avoidance ? object model (yolov5su)
# traffic_light/sign  ? traffic model (yolo11s)
# lane_detection      ? neither (pure CV, no YOLO)
APP_MODEL_MAP = {
    "collision_avoidance":    "object",
    "traffic_light_detection":"traffic",
    "traffic_sign_detection": "traffic",
    "lane_detection":         None,       # pure CV ? gets its own CV timing
}

# ALB Thresholds
RESOURCE_THRESHOLD_PERCENT   = 40
BANDWIDTH_HIGH_THRESHOLD_MBPS = 10
BANDWIDTH_LOW_THRESHOLD_MBPS  = 5

# Edge server
EDGE_SERVER_URL     = "http://192.168.20.16:30052/processimage"
EDGE_TIMEOUT_SEC    = 3.0
EDGE_CHECK_INTERVAL_SEC = 5.0

# Vehicle control
TARGET_SPEED_KMH   = 30
SAFE_DISTANCE_M    = 3.0
TARGET_VEHICLE_LABEL = "car_1"
MAX_THROTTLE       = 0.8

# Image processing
FRAME_WIDTH        = 640
FRAME_HEIGHT       = 480
YOLO_CONF_THRESHOLD = 0.4
ARTIFICIAL_DELAY_MS = 0

# Active applications
ACTIVE_APPLICATIONS = [
    'collision_avoidance',
    'traffic_light_detection',
    'traffic_sign_detection',
]

# YOLO class mappings
TRAFFIC_SIGN_CLASSES  = ['stop sign', 'traffic sign']
TRAFFIC_LIGHT_CLASSES = ['traffic light']
VEHICLE_CLASSES       = ['car', 'truck', 'bus']
PERSON_CLASSES        = ['person']

# Colors
class Color:
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    RED     = '\033[91m'
    BLUE    = '\033[94m'
    CYAN    = '\033[96m'
    MAGENTA = '\033[95m'
    RESET   = '\033[0m'
    BOLD    = '\033[1m'

# ============================================================================
# BACKGROUND RESOURCE MONITOR
# ============================================================================

class BackgroundResourceMonitor:
    """Background thread that continuously monitors resources."""

    def __init__(self, interface=NETWORK_INTERFACE, update_rate=10):
        self.interface       = interface
        self.update_interval = 1.0 / update_rate
        self.lock            = threading.Lock()
        self.cpu_percent     = 0.0
        self.gpu_percent     = 0.0
        self.ram_percent     = 0.0
        self.tx_mbps         = 0.0
        self.rx_mbps         = 0.0
        self.last_net_counters = None
        self.last_net_time   = time.time()
        self.running         = True
        self.thread          = None

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
                'tx_mbps':     self.tx_mbps,
                'rx_mbps':     self.rx_mbps,
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
                    self.tx_mbps     = tx
                    self.rx_mbps     = rx
                elapsed    = time.time() - loop_start
                sleep_time = max(0, self.update_interval - elapsed)
                time.sleep(sleep_time)
            except Exception:
                time.sleep(0.1)

    def _get_gpu_usage(self):
        try:
            result = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
                universal_newlines=True,
            )
            return int(result.strip().split("\n")[0])
        except Exception:
            return 0

    def _get_network_bandwidth(self):
        try:
            net = psutil.net_io_counters(pernic=True)
            if self.interface not in net:
                return 0.0, 0.0
            current = net[self.interface]
            now     = time.time()
            if self.last_net_counters is None:
                self.last_net_counters = current
                self.last_net_time     = now
                return 0.0, 0.0
            interval = now - self.last_net_time
            if interval <= 0:
                return 0.0, 0.0
            tx_mbps = (current.bytes_sent - self.last_net_counters.bytes_sent) * 8 / (interval * 1_048_576)
            rx_mbps = (current.bytes_recv - self.last_net_counters.bytes_recv) * 8 / (interval * 1_048_576)
            self.last_net_counters = current
            self.last_net_time     = now
            return round(tx_mbps, 2), round(rx_mbps, 2)
        except Exception:
            return 0.0, 0.0

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_application_requirements(app_name):
    for app in APPLICATION_TABLE:
        if app["application"] == app_name:
            return {
                "latency_sensitivity": app["latency_sensitivity"],
                "accuracy_priority":   app["accuracy_priority"],
            }
    return None

def get_speed_from_odom(odom_msg):
    if odom_msg is None:
        return 0.0
    vel = odom_msg.twist.twist.linear
    return math.sqrt(vel.x ** 2 + vel.y ** 2) * 3.6

# ============================================================================
# TRUE ADAPTIVE LOAD BALANCER V5.1 ? PER-APP INDEPENDENT LATENCY
# ============================================================================

class TrueAdaptiveLoadBalancerV5:
    """
    TRUE ALB V5.1:
      ? Independent per-app decision
      ? Onboard: each app timed against the specific model it actually uses
      ? Edge: each app timed from its own request dispatch to its own response
      ? GPU compute/idle tracked independently per app
    """

    def __init__(self):
        print(f"\n{Color.BOLD}{Color.CYAN}{'='*70}")
        print(f"  TRUE ALB V5.1 ? INDEPENDENT PER-APP LATENCY + GPU TRACKING")
        print(f"{'='*70}{Color.RESET}\n")

        rospy.init_node('true_alb_v5', anonymous=True)

        self.is_running   = True
        self.frame_id     = 0

        # Image data
        self.rgb_image    = None
        self.depth_image  = None
        self.display_frame = None

        # ----------------------------------------------------------------
        # Cached YOLO results ? run each model AT MOST ONCE per frame
        # We store the raw timing so individual apps can claim their share
        # ----------------------------------------------------------------
        self.cache = {
            # frame_id when cache was last populated
            "frame_id": -1,
            # object model (yolov5su)
            "object_results":      None,
            "object_infer_ms":     0.0,
            "object_gpu_compute_ms": 0.0,
            "object_gpu_idle_ms":  0.0,
            # traffic model (yolo11s)
            "traffic_results":     None,
            "traffic_infer_ms":    0.0,
            "traffic_gpu_compute_ms": 0.0,
            "traffic_gpu_idle_ms": 0.0,
        }

        # Detection results
        self.detection_results = []
        self.target_distance   = None
        self.min_distance      = None

        # Vehicle state
        self.ego_odom          = None
        self.ego_moving        = False
        self.collision_detected = False
        self.collision_actor_id = None

        self.lock = threading.Lock()

        # Edge server
        self.edge_available   = True
        self.last_edge_check  = 0

        # Thread pool ? one future per edge app
        self.edge_executor = ThreadPoolExecutor(max_workers=4)

        # ---- background resource monitor ----
        print(f"{Color.YELLOW}[INIT] Starting background resource monitor...{Color.RESET}")
        self.resource_monitor = BackgroundResourceMonitor(NETWORK_INTERFACE, 10)
        self.resource_monitor.start()
        time.sleep(0.5)
        print(f"{Color.GREEN}[INIT] ? Resource monitor active{Color.RESET}\n")

        # ---- YOLO models ----
        print(f"{Color.YELLOW}[INIT] Loading YOLO models...{Color.RESET}")

        t0 = time.time()
        self.yolo_object_model = YOLO(YOLO_OBJECT_MODEL_PATH)
        if torch.cuda.is_available():
            self.yolo_object_model.to('cuda')
        print(f"{Color.GREEN}[INIT]   ? Object model: {(time.time()-t0)*1000:.1f}ms{Color.RESET}")

        t0 = time.time()
        self.yolo_traffic_model = YOLO(YOLO_TRAFFIC_MODEL_PATH)
        if torch.cuda.is_available():
            self.yolo_traffic_model.to('cuda')
        print(f"{Color.GREEN}[INIT]   ? Traffic model: {(time.time()-t0)*1000:.1f}ms{Color.RESET}")

        if torch.cuda.is_available():
            print(f"{Color.GREEN}[INIT] ? GPU: {torch.cuda.get_device_name(0)}{Color.RESET}\n")
        else:
            print(f"{Color.YELLOW}[INIT] ? CPU only{Color.RESET}\n")

        # ---- ROS I/O ----
        self.control_pub   = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',
                                             CarlaEgoVehicleControl, queue_size=1)
        self.detection_pub = rospy.Publisher('/alb/detections', String, queue_size=10)

        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image',  Image,
                         self.image_callback,    queue_size=1, buff_size=2**24)
        rospy.Subscriber('/carla/ego_vehicle/depth_front/image', Image,
                         self.depth_callback,    queue_size=1, buff_size=2**24)
        rospy.Subscriber('/carla/ego_vehicle/odometry',  Odometry,           self.odometry_callback)
        rospy.Subscriber('/carla/ego_vehicle/collision', CarlaCollisionEvent, self.collision_callback)

        # ---- CSV ----
        print(f"{Color.YELLOW}[INIT] CSV: {CSV_FILE}{Color.RESET}")
        self.csv_file   = open(CSV_FILE, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "frame_id", "application_type", "processing_location",
            "cpu", "gpu", "ram", "tx_mbps", "rx_mbps",
            "edge_available",
            # ?? per-app independent latency ??????????????????????????????
            "app_inference_ms",   # wall time for THIS app's processing
            "gpu_compute_ms",     # GPU kernel time for THIS app
            "gpu_idle_ms",        # non-compute overhead for THIS app
            # ?? frame-level overhead (shared, logged per row) ????????????
            "resource_check_ms", "decision_ms",
            "app_e2e_ms",        # THIS app's true end-to-end (= app_inference_ms, named clearly)
            "frame_total_ms",    # whole frame wall clock (same for all 4 apps in a frame)
            # ?? vehicle state ????????????????????????????????????????????
            "vehicle_speed_kmh", "target_distance_m", "min_distance_m",
            "num_detections", "collision_detected", "collision_actor_id",
        ])
        self.csv_file.flush()
        print(f"{Color.GREEN}[INIT] ? CSV initialized{Color.RESET}\n")

        # ---- signals & threads ----
        signal.signal(signal.SIGINT,  self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

        self.control_thread  = threading.Thread(target=self.control_loop,   daemon=True)
        self.keyboard_thread = threading.Thread(target=self.keyboard_control, daemon=True)
        self.display_thread  = threading.Thread(target=self.display_loop,   daemon=True)
        for t in (self.control_thread, self.keyboard_thread, self.display_thread):
            t.start()

        print(f"{Color.BOLD}{Color.GREEN}{'='*70}")
        print(f"  SYSTEM READY  |  's' = START   'a' = STOP")
        print(f"{'='*70}{Color.RESET}\n")

    # ------------------------------------------------------------------ shutdown
    def shutdown(self, signum=None, frame=None):
        print(f"\n{Color.RED}[SHUTDOWN] Stopping...{Color.RESET}")
        self.is_running = False
        if hasattr(self, 'edge_executor'):     self.edge_executor.shutdown(wait=False)
        if hasattr(self, 'resource_monitor'):  self.resource_monitor.stop()
        if hasattr(self, 'csv_file'):          self.csv_file.close()
        if hasattr(self, 'control_pub'):
            msg = CarlaEgoVehicleControl(); msg.throttle = 0.0; msg.brake = 1.0
            self.control_pub.publish(msg)
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown")
        print(f"{Color.GREEN}[SHUTDOWN] ? Done{Color.RESET}\n")

    # ================================================================= ROS callbacks
    def image_callback(self, ros_image):
        try:
            frame_rx_time    = time.time()
            current_frame_id = self.frame_id

            print(f"\n{Color.CYAN}{'?'*70}")
            print(f"[FRAME {current_frame_id}] Received")
            print(f"{'?'*70}{Color.RESET}")

            if ARTIFICIAL_DELAY_MS > 0:
                time.sleep(ARTIFICIAL_DELAY_MS / 1000.0)

            if ros_image.encoding not in ['bgra8', 'bgr8']:
                return

            img_np = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                (ros_image.height, ros_image.width, -1))
            frame  = img_np[:, :, :3]
            frame  = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            with self.lock:
                self.rgb_image = frame

            self.handle_frame(frame, current_frame_id, frame_rx_time)
            self.frame_id += 1

        except Exception as e:
            rospy.logerr(f"[IMAGE] Error: {e}")
            import traceback; traceback.print_exc()

    def depth_callback(self, ros_image):
        try:
            depth_np = np.frombuffer(ros_image.data, dtype=np.float32).reshape(
                (ros_image.height, ros_image.width))
            with self.lock:
                self.depth_image = depth_np
        except Exception:
            pass

    def odometry_callback(self, msg):  self.ego_odom = msg

    def collision_callback(self, msg):
        if hasattr(msg, 'other_actor_id') and msg.other_actor_id != 0:
            self.collision_detected  = True
            self.collision_actor_id  = msg.other_actor_id
            print(f"{Color.RED}[COLLISION] Actor {msg.other_actor_id}!{Color.RESET}")
        else:
            self.collision_detected  = False
            self.collision_actor_id  = None

    # ================================================================= CORE ALB
    def handle_frame(self, frame, frame_id, frame_rx_time):
        """
        Per-frame entry point.
        Produces a dict  app_metrics[app] = {
            'location':        str,
            'inference_ms':    float,   ? THIS app's individual wall time
            'gpu_compute_ms':  float,   ? THIS app's GPU kernel time
            'gpu_idle_ms':     float,   ? THIS app's non-compute overhead
        }
        """
        try:
            # ?? 1. Resources (once per frame) ???????????????????????????????
            t0        = time.time()
            resources = self.resource_monitor.get_resources()
            edge_avail = self.check_edge_server_available()
            resource_check_ms = (time.time() - t0) * 1000.0

            print(f"\n{Color.BLUE}[RES] CPU {resources['cpu_percent']:.1f}% | "
                  f"GPU {resources['gpu_percent']:.1f}% | RAM {resources['ram_percent']:.1f}% | "
                  f"TX {resources['tx_mbps']:.2f}Mbps | Edge {'?' if edge_avail else '?'}{Color.RESET}")

            # ?? 2. Per-app ALB decisions ?????????????????????????????????????
            # app_pipeline_start[app] = wall time when THIS app's pipeline begins
            # (frame_rx_time + resource_check + all prior decisions)
            # Used to compute true per-app E2E: frame receipt ? result ready
            app_decisions          = {}
            app_pipeline_start     = {}
            total_decision_ms      = 0.0
            cumulative_decision_ms = 0.0

            for app in ACTIVE_APPLICATIONS:
                req = get_application_requirements(app)
                if not req:
                    continue
                t0 = time.time()
                loc = self.decide_processing_location(
                    req["latency_sensitivity"], req["accuracy_priority"],
                    resources['cpu_percent'], resources['ram_percent'],
                    resources['gpu_percent'], resources['tx_mbps'],
                    resources['rx_mbps'], edge_avail,
                )
                this_decision_ms        = (time.time() - t0) * 1000.0
                total_decision_ms      += this_decision_ms
                cumulative_decision_ms += this_decision_ms

                # Pipeline starts after resource_check + decisions up to this app
                app_pipeline_start[app] = (
                    frame_rx_time
                    + (resource_check_ms / 1000.0)
                    + (cumulative_decision_ms / 1000.0)
                )

                app_decisions[app] = loc
                print(f"{Color.YELLOW}[ALB] {app:30s} ? {loc.upper()}{Color.RESET}")

            # ?? 3. Group by location ?????????????????????????????????????????
            onboard_apps      = [a for a, l in app_decisions.items() if l == "onboard"]
            edge_apps         = [a for a, l in app_decisions.items() if l == "edge"]
            edge_lowres_apps  = [a for a, l in app_decisions.items() if l == "edge_lowres"]

            print(f"\n{Color.MAGENTA}[PLAN] Onboard:{onboard_apps}  "
                  f"Edge:{edge_apps}  EdgeLow:{edge_lowres_apps}{Color.RESET}")

            # ?? 4. Run onboard models (with cache) ? returns per-app metrics ?
            app_metrics: dict = {}

            if onboard_apps:
                onboard_metrics = self._process_onboard_apps(frame, frame_id, onboard_apps, app_pipeline_start)
                app_metrics.update(onboard_metrics)

            # ?? 5. Edge apps ? each gets its OWN future ? own timing ?????????
            if edge_apps:
                edge_metrics = self._process_edge_apps(frame, frame_id, edge_apps, app_pipeline_start)
                app_metrics.update(edge_metrics)

            if edge_lowres_apps:
                lowres = cv2.resize(frame, (320, 240))
                lowres_metrics = self._process_edge_apps(lowres, frame_id, edge_lowres_apps, app_pipeline_start)
                app_metrics.update(lowres_metrics)

            # ?? 6. Summary ???????????????????????????????????????????????????
            total_ms    = (time.time() - frame_rx_time) * 1000.0
            speed_kmh   = get_speed_from_odom(self.ego_odom)
            num_det     = len(self.detection_results)

            print(f"\n{Color.BOLD}{Color.GREEN}?{'?'*60}?")
            print(f"?  FRAME {frame_id} SUMMARY  (frame_total={total_ms:.1f}ms)")
            print(f"?{'?'*60}?")
            for app in ACTIVE_APPLICATIONS:
                m = app_metrics.get(app, {})
                print(f"?  {app:30s}  [{m.get('location','?'):12s}]")
                print(f"?    app_e2e:   {m.get('app_e2e_ms',0):7.2f}ms  "
                      f"inference: {m.get('inference_ms',0):7.2f}ms  "
                      f"gpu_c: {m.get('gpu_compute_ms',0):6.2f}ms")
            print(f"?{'?'*60}?{Color.RESET}")

            # ?? 7. CSV ? one row per app with its own latency numbers ????????
            for app in ACTIVE_APPLICATIONS:
                m    = app_metrics.get(app, {})
                loc  = app_decisions.get(app, "unknown")
                self.log_to_csv(
                    timestamp         = time.time(),
                    frame_id          = frame_id,
                    app_type          = app,
                    location          = loc,
                    cpu               = resources['cpu_percent'],
                    gpu               = resources['gpu_percent'],
                    ram               = resources['ram_percent'],
                    tx_mbps           = resources['tx_mbps'],
                    rx_mbps           = resources['rx_mbps'],
                    edge_available    = 1 if edge_avail else 0,
                    app_inference_ms  = m.get('inference_ms',  0.0),
                    gpu_compute_ms    = m.get('gpu_compute_ms', 0.0),
                    gpu_idle_ms       = m.get('gpu_idle_ms',    0.0),
                    resource_check_ms = resource_check_ms / len(ACTIVE_APPLICATIONS),
                    decision_ms       = total_decision_ms  / len(ACTIVE_APPLICATIONS),
                    app_e2e_ms        = m.get('app_e2e_ms', 0.0),  # frame_rx ? result ready
                    frame_total_ms    = total_ms,
                    speed_kmh         = speed_kmh,
                    target_dist       = self.target_distance,
                    min_dist          = self.min_distance,
                    num_detections    = num_det,
                    collision         = 1 if self.collision_detected else 0,
                    collision_id      = self.collision_actor_id,
                )

        except Exception as e:
            rospy.logerr(f"[ALB] Error: {e}")
            import traceback; traceback.print_exc()

    # ================================================================= ALB decision
    def decide_processing_location(self, latency_sens, accuracy_pri,
                                   cpu, ram, gpu, tx_mbps, rx_mbps, edge_avail):
        onboard_ok = (cpu < RESOURCE_THRESHOLD_PERCENT and gpu < RESOURCE_THRESHOLD_PERCENT)
        bw_high    = (tx_mbps >= BANDWIDTH_HIGH_THRESHOLD_MBPS)
        bw_low     = (tx_mbps <  BANDWIDTH_LOW_THRESHOLD_MBPS)
        bw_zero    = (tx_mbps == 0)

        if not edge_avail:
            return "onboard"

        if latency_sens == "high":
            return "onboard" if onboard_ok or not bw_high else "edge"
        else:   # accuracy high
            if bw_high:   return "edge"
            else:         return "onboard"

    def check_edge_server_available(self):
        now = time.time()
        if now - self.last_edge_check < EDGE_CHECK_INTERVAL_SEC:
            return self.edge_available
        try:
            resp = requests.head(EDGE_SERVER_URL, timeout=2.0)
            self.edge_available = resp.status_code < 500
        except Exception:
            self.edge_available = False
        self.last_edge_check = now
        return self.edge_available

    # ================================================================= ONBOARD
    def _run_model_with_gpu_timing(self, model, frame):
        """
        Run ONE model inference with CUDA event timing.
        Returns (results, infer_wall_ms, gpu_compute_ms, gpu_idle_ms).
        """
        use_cuda = torch.cuda.is_available()

        if use_cuda:
            torch.cuda.synchronize()
            ev_start = torch.cuda.Event(enable_timing=True)
            ev_end   = torch.cuda.Event(enable_timing=True)
            ev_start.record()

        t_wall = time.time()
        results = model.predict(frame, conf=YOLO_CONF_THRESHOLD, verbose=False)[0]
        infer_wall_ms = (time.time() - t_wall) * 1000.0

        if use_cuda:
            ev_end.record()
            torch.cuda.synchronize()
            gpu_compute_ms = ev_start.elapsed_time(ev_end)
        else:
            gpu_compute_ms = 0.0

        gpu_idle_ms = max(0.0, infer_wall_ms - gpu_compute_ms)
        return results, infer_wall_ms, gpu_compute_ms, gpu_idle_ms

    def _ensure_object_model_cached(self, frame, frame_id):
        """Run yolov5su if not already cached for this frame."""
        if self.cache["frame_id"] == frame_id and self.cache["object_results"] is not None:
            print(f"{Color.GREEN}[ONBOARD] ? Using cached yolov5su{Color.RESET}")
            return
        print(f"{Color.GREEN}[ONBOARD] Running yolov5su?{Color.RESET}")
        results, wall, gpu_c, gpu_i = self._run_model_with_gpu_timing(self.yolo_object_model, frame)
        self.cache["object_results"]       = results
        self.cache["object_infer_ms"]      = wall
        self.cache["object_gpu_compute_ms"]= gpu_c
        self.cache["object_gpu_idle_ms"]   = gpu_i
        print(f"{Color.BLUE}[GPU] yolov5su  wall={wall:.1f}ms  compute={gpu_c:.1f}ms  idle={gpu_i:.1f}ms{Color.RESET}")

    def _ensure_traffic_model_cached(self, frame, frame_id):
        """Run yolo11s if not already cached for this frame."""
        if self.cache["frame_id"] == frame_id and self.cache["traffic_results"] is not None:
            print(f"{Color.GREEN}[ONBOARD] ? Using cached yolo11s{Color.RESET}")
            return
        print(f"{Color.GREEN}[ONBOARD] Running yolo11s?{Color.RESET}")
        results, wall, gpu_c, gpu_i = self._run_model_with_gpu_timing(self.yolo_traffic_model, frame)
        self.cache["traffic_results"]       = results
        self.cache["traffic_infer_ms"]      = wall
        self.cache["traffic_gpu_compute_ms"]= gpu_c
        self.cache["traffic_gpu_idle_ms"]   = gpu_i
        print(f"{Color.BLUE}[GPU] yolo11s   wall={wall:.1f}ms  compute={gpu_c:.1f}ms  idle={gpu_i:.1f}ms{Color.RESET}")

    def _process_onboard_apps(self, frame, frame_id, apps, app_pipeline_start):
        """
        Run required models (at most once each via cache), then assign
        each app its OWN timing based on which model it actually uses.

        app_e2e_ms = time from frame_rx ? THIS app's result ready
          Measured as: (result_ready_wall_time - app_pipeline_start[app]) * 1000
          where app_pipeline_start already includes frame_rx + resource_check + decision overhead

        Returns dict: app ? {location, inference_ms, gpu_compute_ms, gpu_idle_ms, app_e2e_ms}
        """
        need_object  = any(APP_MODEL_MAP.get(a) == "object"  for a in apps)
        need_traffic = any(APP_MODEL_MAP.get(a) == "traffic" for a in apps)
        need_lane    = any(APP_MODEL_MAP.get(a) is None       for a in apps)

        # Run models (cache hit = free). Record wall-clock moment each finishes.
        if need_object:
            self._ensure_object_model_cached(frame, frame_id)
        t_object_done = time.time()

        if need_traffic:
            self._ensure_traffic_model_cached(frame, frame_id)
        t_traffic_done = time.time()

        self.cache["frame_id"] = frame_id

        # Lane detection ? pure CV
        lane_wall_ms = 0.0
        t_lane_done  = t_traffic_done
        if need_lane:
            t0 = time.time()
            self._run_lane_detection_cv(frame)
            lane_wall_ms = (time.time() - t0) * 1000.0
            t_lane_done  = time.time()

        # Build per-app metrics
        metrics    = {}
        detections = []
        car_count  = 0

        for app in apps:
            model_key    = APP_MODEL_MAP.get(app)
            pipeline_start = app_pipeline_start.get(app, t_object_done)

            if model_key == "object":
                infer_ms = self.cache["object_infer_ms"]
                gpu_c_ms = self.cache["object_gpu_compute_ms"]
                gpu_i_ms = self.cache["object_gpu_idle_ms"]

                # Parse detections + depth (collision_avoidance needs this before deciding)
                results = self.cache["object_results"]
                for r in results.boxes:
                    conf    = float(r.conf[0])
                    cls_idx = int(r.cls[0])
                    label   = results.names[cls_idx].lower()
                    bbox    = r.xyxy[0].tolist()
                    if any(v in label for v in VEHICLE_CLASSES) or label in PERSON_CLASSES:
                        car_count += 1
                        detections.append({
                            'bbox':         bbox,
                            'class':        label,
                            'confidence':   conf,
                            'unique_label': f"{label}_{car_count}",
                            'distance':     self._calculate_distance(bbox),
                        })
                # E2E: decision done ? model ran ? depth parsed ? result ready NOW
                app_e2e_ms = (time.time() - pipeline_start) * 1000.0

            elif model_key == "traffic":
                infer_ms   = self.cache["traffic_infer_ms"]
                gpu_c_ms   = self.cache["traffic_gpu_compute_ms"]
                gpu_i_ms   = self.cache["traffic_gpu_idle_ms"]
                # E2E: decision done ? traffic model done
                app_e2e_ms = (t_traffic_done - pipeline_start) * 1000.0

            else:  # lane detection (pure CV)
                infer_ms   = lane_wall_ms
                gpu_c_ms   = 0.0
                gpu_i_ms   = lane_wall_ms
                # E2E: decision done ? CV done
                app_e2e_ms = (t_lane_done - pipeline_start) * 1000.0

            metrics[app] = {
                'location':       'onboard',
                'inference_ms':   infer_ms,
                'gpu_compute_ms': gpu_c_ms,
                'gpu_idle_ms':    gpu_i_ms,
                'app_e2e_ms':     app_e2e_ms,   # ? true per-app E2E from frame arrival
            }
            print(f"{Color.GREEN}[ONBOARD] {app:30s}  "
                  f"inf={infer_ms:.1f}ms  e2e={app_e2e_ms:.1f}ms  "
                  f"gpu_c={gpu_c_ms:.1f}ms  gpu_i={gpu_i_ms:.1f}ms{Color.RESET}")

        # Store detection results
        with self.lock:
            self.detection_results = detections
            self.target_distance   = next(
                (d['distance'] for d in detections if d['unique_label'] == TARGET_VEHICLE_LABEL), None)
            dists = [d['distance'] for d in detections if d['distance'] is not None]
            self.min_distance = min(dists) if dists else None

        self.detection_pub.publish(String(data=json.dumps(detections)))
        self.update_display(frame, detections)

        return metrics

    def _run_lane_detection_cv(self, frame):
        """Placeholder for pure-CV lane detection (no YOLO)."""
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges  = cv2.Canny(gray, 50, 150)
        # (actual lane line fitting would go here)
        return edges

    # ================================================================= EDGE
    def _process_edge_apps(self, frame, frame_id, apps, app_pipeline_start):
        """
        Send each app as its OWN HTTP request.
        app_e2e_ms = app_pipeline_start[app] ? that app's response fully processed.
        Each app's pipeline_start already accounts for frame_rx + resource_check + its decision.
        """
        use_cuda = torch.cuda.is_available()
        if use_cuda:
            torch.cuda.synchronize()
        t_enc   = time.time()
        success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if use_cuda:
            torch.cuda.synchronize()
        encode_gpu_ms = (time.time() - t_enc) * 1000.0

        if not success:
            return {a: {'location':'edge','inference_ms':0,'gpu_compute_ms':0,
                        'gpu_idle_ms':0,'app_e2e_ms':0} for a in apps}

        img_bytes = buffer.tobytes()

        futures = {
            self.edge_executor.submit(
                self._send_single_edge_request,
                img_bytes, frame.shape, frame_id, app, frame,
                encode_gpu_ms, app_pipeline_start.get(app, time.time())
            ): app
            for app in apps
        }

        metrics = {}
        for future in as_completed(futures, timeout=EDGE_TIMEOUT_SEC + 1):
            app = futures[future]
            try:
                m = future.result()
            except Exception as e:
                print(f"{Color.RED}[EDGE] {app} failed: {e}{Color.RESET}")
                m = {'location':'edge','inference_ms':0,'gpu_compute_ms':encode_gpu_ms,
                     'gpu_idle_ms':0,'app_e2e_ms':0}
            metrics[app] = m
            print(f"{Color.CYAN}[EDGE] {app:30s}  "
                  f"app_e2e={m['app_e2e_ms']:.1f}ms  net={m['inference_ms']:.1f}ms  "
                  f"gpu_c={m['gpu_compute_ms']:.1f}ms{Color.RESET}")

        return metrics

    def _send_single_edge_request(self, img_bytes, frame_shape, frame_id, app, frame,
                                   encode_gpu_ms, pipeline_start):
        """
        Send ONE request for ONE app.

        pipeline_start = wall time when THIS app's pipeline began
                         (frame_rx + resource_check + this app's decision)

        app_e2e_ms = time.time() - pipeline_start  (stamped when result is ready)

        collision_avoidance:  pipeline_start ? encode ? send ? receive ? depth_parse
        other edge apps:      pipeline_start ? encode ? send ? receive
        """
        headers = {
            'Content-Type':     'application/octet-stream',
            'Frame-Width':      str(frame_shape[1]),
            'Frame-Height':     str(frame_shape[0]),
            'Frame-Identifier': f"frame_{frame_id}",
            'detection-flag':   app,
        }

        t_req_start = time.time()
        resp        = requests.post(EDGE_SERVER_URL,
                                    data=img_bytes, headers=headers,
                                    timeout=EDGE_TIMEOUT_SEC)
        t_req_end   = time.time()
        network_ms  = (t_req_end - t_req_start) * 1000.0

        if resp.status_code != 200:
            app_e2e_ms = (time.time() - pipeline_start) * 1000.0
            return {
                'location':       'edge',
                'inference_ms':   network_ms,
                'gpu_compute_ms': encode_gpu_ms,
                'gpu_idle_ms':    network_ms,
                'app_e2e_ms':     app_e2e_ms,
            }

        if app == 'collision_avoidance':
            try:
                result = resp.json()
                if 'OBD_model' in result:
                    self._parse_collision_detections(result, frame)
            except Exception:
                pass
            # E2E ends after local depth parse completes
            app_e2e_ms  = (time.time() - pipeline_start) * 1000.0
            gpu_idle_ms = network_ms + (app_e2e_ms - encode_gpu_ms - network_ms)
        else:
            # E2E ends when response received ? no local post-processing
            app_e2e_ms  = (t_req_end - pipeline_start) * 1000.0
            gpu_idle_ms = network_ms

        return {
            'location':       'edge',
            'inference_ms':   network_ms,
            'gpu_compute_ms': encode_gpu_ms,
            'gpu_idle_ms':    gpu_idle_ms,
            'app_e2e_ms':     app_e2e_ms,   # ? pipeline_start ? THIS app's result ready
        }

    def _parse_collision_detections(self, result, frame):
        try:
            obd_data    = result['OBD_model']
            if isinstance(obd_data, str):
                obd_data = json.loads(obd_data)
            raw         = obd_data.get('detections', [])
            detections  = []
            car_count   = 0
            for det in raw:
                if det.get("class", "").lower() in ['car', 'truck', 'bus']:
                    car_count += 1
                    det['unique_label'] = f"{det.get('class')}_{car_count}"
                    det['distance']     = self._calculate_distance(det.get("bbox"))
                    detections.append(det)
            with self.lock:
                self.detection_results = detections
                self.target_distance   = next(
                    (d['distance'] for d in detections if d.get('unique_label') == TARGET_VEHICLE_LABEL), None)
                dists = [d.get('distance') for d in detections if d.get('distance') is not None]
                self.min_distance = min(dists) if dists else None
            self.detection_pub.publish(String(data=json.dumps(detections)))
            self.update_display(frame, detections)
        except Exception:
            pass

    def _calculate_distance(self, bbox):
        if self.depth_image is None or not bbox:
            return None
        try:
            x1, y1, x2, y2 = map(int, bbox)
            H, W = self.depth_image.shape
            x1, x2 = max(0,x1), min(W-1,x2)
            y1, y2 = max(0,y1), min(H-1,y2)
            if x2 > x1 and y2 > y1:
                patch = self.depth_image[y1:y2, x1:x2]
                if patch.size > 0:
                    return float(np.median(patch))
        except Exception:
            pass
        return None

    # ================================================================= CONTROL
    def control_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.is_running:
            try:     self.control_vehicle()
            except Exception: pass
            rate.sleep()

    def control_vehicle(self):
        msg = CarlaEgoVehicleControl()
        msg.steer = 0.0; msg.hand_brake = False; msg.reverse = False
        speed = get_speed_from_odom(self.ego_odom)
        if not self.ego_moving:
            msg.throttle = 0.0; msg.brake = 1.0
        elif self.target_distance and self.target_distance <= SAFE_DISTANCE_M:
            msg.throttle = 0.0; msg.brake = 1.0
        elif speed < TARGET_SPEED_KMH:
            msg.throttle = MAX_THROTTLE; msg.brake = 0.0
        else:
            msg.throttle = 0.0; msg.brake = 0.0
        self.control_pub.publish(msg)

    def keyboard_control(self):
        def on_press(key):
            try:
                if hasattr(key, 'char'):
                    if   key.char == 's': self.ego_moving = True;  print(f"\n{Color.GREEN}[KEY] START{Color.RESET}")
                    elif key.char == 'a': self.ego_moving = False; print(f"\n{Color.RED}[KEY] STOP{Color.RESET}")
            except Exception: pass
        from pynput import keyboard as kb
        kb.Listener(on_press=on_press).run()

    # ================================================================= DISPLAY
    def update_display(self, frame, detections):
        display = frame.copy()
        for det in detections:
            bbox = det.get("bbox")
            if not bbox: continue
            x1,y1,x2,y2 = map(int, bbox)
            dist  = det.get("distance")
            color = (0,0,255) if (dist and dist <= SAFE_DISTANCE_M) else (0,255,0)
            cv2.rectangle(display, (x1,y1),(x2,y2), color, 2)
            text  = det.get("unique_label","")
            if dist: text += f" {dist:.2f}m"
            cv2.putText(display, text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        with self.lock:
            self.display_frame = display

    def display_loop(self):
        rate = rospy.Rate(30)
        while self.is_running and not rospy.is_shutdown():
            if self.display_frame is not None:
                cv2.imshow("ALB V5.1", self.display_frame)
                cv2.waitKey(1)
            rate.sleep()

    # ================================================================= CSV
    def log_to_csv(self, timestamp, frame_id, app_type, location, cpu, gpu, ram,
                   tx_mbps, rx_mbps, edge_available, app_inference_ms, gpu_compute_ms,
                   gpu_idle_ms, resource_check_ms, decision_ms, app_e2e_ms, frame_total_ms,
                   speed_kmh, target_dist, min_dist, num_detections, collision, collision_id):
        self.csv_writer.writerow([
            timestamp, frame_id, app_type, location,
            cpu, gpu, ram, tx_mbps, rx_mbps, edge_available,
            round(app_inference_ms,  3),
            round(gpu_compute_ms,    3),
            round(gpu_idle_ms,       3),
            round(resource_check_ms, 3),
            round(decision_ms,       3),
            round(app_e2e_ms,        3),   # per-app ? differs across all 4 rows
            round(frame_total_ms,    3),   # shared frame wall clock
            round(speed_kmh,         2),
            target_dist if target_dist is not None else "",
            min_dist    if min_dist    is not None else "",
            num_detections, collision,
            collision_id if collision_id is not None else "",
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
        import traceback; traceback.print_exc()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
