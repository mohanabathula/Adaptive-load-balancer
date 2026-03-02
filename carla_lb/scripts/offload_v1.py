#!/usr/bin/env python3
"""
Optimized Pure Edge Offload System - ALL 4 APPS + GPU Tracking
===============================================================
FIX: gpu_compute_ms is always 0.0 for edge ? edge does NO local GPU work.
     Removed torch.cuda.synchronize() from encoding path entirely.
     The old code wrapped CPU-only cv2 calls between two synchronize()
     calls, which drained the GPU pipeline and inflated gpu_compute_ms
     by 10-50ms. Edge node never runs YOLO locally ? gpu_compute = 0.

Author: Vidya Vepoori
Date: 2026-02-06
Version: FIXED - CORRECT GPU TRACKING FOR EDGE
"""

import os
import rospy
import cv2
import json
import time
import csv
import threading
import numpy as np
import requests
import psutil
import math
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaCollisionEvent
from cv_bridge import CvBridge
from pynput import keyboard
from pynvml import nvmlInit, nvmlShutdown, nvmlDeviceGetHandleByIndex, nvmlDeviceGetUtilizationRates
from queue import Queue

# ============================================================================
# CONFIGURATION
# ============================================================================

CSV_FILE = "/media/root/b9f388cb-81d8-4121-93fb-ad514e324552/vidyav/main/edge_plot.csv"
INTERFACE = "enp25s0f0"
GPU_INDEX = 0
ARTIFICIAL_DELAY_MS = 0

TARGET_SPEED_KMH     = 30
SAFE_DISTANCE_M      = 3.0
TARGET_VEHICLE_LABEL = "car_1"
MAX_THROTTLE         = 0.8

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
# RESOURCE MONITOR  (background, non-blocking)
# ============================================================================

class ResourceMonitor:
    """
    Background resource monitor @ 2Hz using pynvml.

    GPU NOTE ? why edge 'gpu' column reads high:
    ??????????????????????????????????????????????
    nvmlDeviceGetUtilizationRates() returns SYSTEM-WIDE GPU utilisation,
    not per-process. CARLA's renderer runs on the same GPU and is always
    active ? 'system_gpu' will be high even though this script does zero
    GPU inference.

    We now log TWO separate GPU fields in CSV:
      system_gpu_pct  ? whole-machine GPU % (CARLA renderer included)
      proc_gpu_mb     ? GPU memory used by THIS process only (~0 MB for edge)

    This makes it clear: edge process itself uses 0 MB GPU, system is
    high only because of CARLA.

    Polling reduced 10Hz ? 2Hz to cut monitor's own CPU overhead.
    """

    def __init__(self, gpu_index=0, interfaces=None):
        self.interfaces         = interfaces or []
        nvmlInit()
        self.gpu_handle         = nvmlDeviceGetHandleByIndex(gpu_index)
        self.prev_net           = psutil.net_io_counters(pernic=True)
        self.prev_time          = time.time()
        self.latest_cpu         = 0.0
        self.latest_system_gpu  = 0      # whole-machine GPU % (CARLA included)
        self.latest_proc_gpu_mb = 0.0   # THIS process GPU memory only (~0 for edge)
        self.latest_ram         = 0.0
        self.latest_tx_mbps     = 0.0
        self.latest_rx_mbps     = 0.0
        self._pid               = os.getpid()
        self.monitoring         = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.monitoring:
            try:
                self.latest_cpu = psutil.cpu_percent(interval=None)
                self.latest_ram = psutil.virtual_memory().percent

                # System-wide GPU % ? CARLA renderer makes this always high
                self.latest_system_gpu = nvmlDeviceGetUtilizationRates(
                    self.gpu_handle).gpu

                # Per-process GPU memory for THIS pid only ? ~0 MB for edge
                self.latest_proc_gpu_mb = self._proc_gpu_mb()

                now      = time.time()
                interval = now - self.prev_time
                net      = psutil.net_io_counters(pernic=True)
                iface    = self.interfaces[0] if self.interfaces else None
                if iface and iface in net and iface in self.prev_net \
                        and interval > 0:
                    self.latest_tx_mbps = (
                        net[iface].bytes_sent -
                        self.prev_net[iface].bytes_sent) * 8 / (1e6 * interval)
                    self.latest_rx_mbps = (
                        net[iface].bytes_recv -
                        self.prev_net[iface].bytes_recv) * 8 / (1e6 * interval)
                self.prev_net  = net
                self.prev_time = now
            except Exception:
                pass
            time.sleep(0.5)   # 2Hz ? reduced from 0.1s (10Hz)

    def _proc_gpu_mb(self):
        """Return MB of GPU memory used by THIS process (pid) only."""
        try:
            from pynvml import nvmlDeviceGetComputeRunningProcesses
            for p in nvmlDeviceGetComputeRunningProcesses(self.gpu_handle):
                if p.pid == self._pid:
                    return round(p.usedGpuMemory / 1024 / 1024, 1)
        except Exception:
            pass
        return 0.0   # edge not in GPU compute list ? correct: 0 MB

    def get(self):
        return {
            'cpu'         : self.latest_cpu,
            'system_gpu'  : self.latest_system_gpu,   # machine-wide (CARLA incl.)
            'proc_gpu_mb' : self.latest_proc_gpu_mb,  # this process only ? ~0
            'ram'         : self.latest_ram,
            'tx_mbps'     : self.latest_tx_mbps,
            'rx_mbps'     : self.latest_rx_mbps,
        }

    def shutdown(self):
        self.monitoring = False
        nvmlShutdown()


# ============================================================================
# ASYNC CSV WRITER
# ============================================================================

class AsyncCSVWriter:
    def __init__(self, filepath, header):
        self.queue   = Queue(maxsize=1000)
        self.file    = open(filepath, 'w', newline='')
        self.writer  = csv.writer(self.file)
        self.writer.writerow(header)
        self.file.flush()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def write_row(self, row):
        try:
            self.queue.put(row, block=False)
        except:
            pass

    def _loop(self):
        batch = []
        while self.running:
            try:
                row = self.queue.get(timeout=0.1)
                batch.append(row)
                if self.queue.qsize() == 0 or len(batch) >= 10:
                    for r in batch:
                        self.writer.writerow(r)
                    self.file.flush()
                    batch = []
            except:
                if batch:
                    for r in batch:
                        self.writer.writerow(r)
                    self.file.flush()
                    batch = []

    def close(self):
        self.running = False
        deadline = time.time() + 2.0
        while not self.queue.empty() and time.time() < deadline:
            time.sleep(0.1)
        self.file.close()


# ============================================================================
# EDGE FORWARDER
# ============================================================================

class EdgeForwarderOptimized:

    def __init__(self):
        rospy.init_node('edge_forwarder_optimized', anonymous=True)

        print(f"\n{Color.BOLD}{Color.CYAN}{'='*80}")
        print(f"  EDGE OFFLOAD ? ALL 4 APPS + FIXED GPU TRACKING")
        print(f"  gpu_compute_ms = 0 (edge does no local GPU work)")
        print(f"  gpu_idle_ms    = pure network round-trip time")
        print(f"{'='*80}{Color.RESET}\n")

        self.server_url   = rospy.get_param('~server_url')
        self.edge_timeout = rospy.get_param('~edge_timeout', 6.0)
        self.frame_width  = rospy.get_param('~frame_width',  640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.image_topic  = rospy.get_param('~image_topic')
        self.depth_topic  = rospy.get_param('~depth_topic')
        self.applications = rospy.get_param('~applications', [
            'collision_avoidance',
            'traffic_light_detection',
            'traffic_sign_detection',
        ])

        self.bridge             = CvBridge()
        self.lock               = threading.Lock()
        self.frame_id           = 0
        self.latest_depth_image = None
        self.app_detections     = {
            'collision_avoidance'     : [],
            'lane_detection'          : [],
            'traffic_light_detection' : [],
            'traffic_sign_detection'  : [],
        }
        self.ego_odom           = None
        self.ego_moving         = False
        self.collision_detected = False
        self.collision_actor_id = None
        self.target_distance    = None
        self.min_distance       = None

        rospy.Subscriber(self.image_topic, Image,
                         self.image_callback, queue_size=10)
        rospy.Subscriber(self.depth_topic, Image,
                         self.depth_callback,  queue_size=10)
        rospy.Subscriber('/carla/ego_vehicle/odometry',
                         Odometry, self.odometry_callback)
        rospy.Subscriber('/carla/ego_vehicle/collision',
                         CarlaCollisionEvent, self.collision_callback)

        self.obj_pub     = rospy.Publisher('/carla/edge_detection',
                                           String, queue_size=1)
        self.control_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd',
            CarlaEgoVehicleControl, queue_size=1)

        print(f"{Color.YELLOW}[INIT] Starting resource monitor...{Color.RESET}")
        self.res_monitor = ResourceMonitor(GPU_INDEX, [INTERFACE])
        print(f"{Color.GREEN}[INIT] ? Resource monitor running (pynvml, no subprocess){Color.RESET}\n")

        csv_header = [
            "timestamp", "frame_id", "app_type", "processing_location",
            "true_e2e_ms",
            "gpu_compute_ms",    # always 0.0 ? edge has no local GPU inference
            "gpu_idle_ms",       # = pure network round-trip (send ? receive)
            "cpu",
            "system_gpu_pct",    # whole-machine GPU % ? HIGH because CARLA renderer
            "proc_gpu_mb",       # THIS process GPU memory only ? ~0 MB for edge
            "ram", "tx_mbps", "rx_mbps",
            "speed_kmh", "target_dist", "min_dist",
            "num_detections", "detection_classes",
            "collision", "collision_id",
        ]
        self.async_csv = AsyncCSVWriter(CSV_FILE, csv_header)
        print(f"{Color.GREEN}[INIT] ? CSV ready{Color.RESET}\n")

        threading.Thread(target=self.control_loop,     daemon=True).start()
        threading.Thread(target=self.keyboard_control, daemon=True).start()

        print(f"{Color.BOLD}{Color.GREEN}{'='*80}")
        print(f"  SYSTEM READY")
        print(f"{'='*80}{Color.RESET}\n")

    # ======================================================================
    # ROS CALLBACKS
    # ======================================================================

    def image_callback(self, msg):
        print(f"\n{Color.CYAN}[FRAME {self.frame_id}] ? {len(self.applications)} apps ? edge{Color.RESET}")
        try:
            t_e2e_start = time.time()

            # ?? Encode frame (pure CPU ? NO cuda.synchronize()) ????????
            # FIX: removed torch.cuda.synchronize() that was draining the
            # GPU pipeline and inflating gpu_compute_ms by 10-50ms.
            # cv2.imencode is CPU-only; no GPU sync needed here.
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            image    = cv2.resize(cv_image, (self.frame_width, self.frame_height))

            # gpu_compute_ms = 0.0 because edge does NO local GPU inference.
            # Encoding is CPU-only. Labelling it "GPU compute" would be wrong.
            gpu_compute_ms = 0.0

            if ARTIFICIAL_DELAY_MS > 0:
                time.sleep(ARTIFICIAL_DELAY_MS / 1000.0)

            for app in self.applications:
                self.forward_to_edge(image, app, t_e2e_start, gpu_compute_ms)

        except Exception as e:
            rospy.logwarn(f"Image callback error: {e}")

    def depth_callback(self, msg):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if d.dtype != np.float32:
                d = d.astype(np.float32)
            with self.lock:
                self.latest_depth_image = d
        except Exception as e:
            rospy.logwarn(f"Depth: {e}")

    def odometry_callback(self, msg):
        self.ego_odom = msg

    def collision_callback(self, msg):
        if hasattr(msg, 'other_actor_id') and msg.other_actor_id != 0:
            self.collision_detected = True
            self.collision_actor_id = msg.other_actor_id
        else:
            self.collision_detected = False
            self.collision_actor_id = None

    # ======================================================================
    # EDGE REQUEST
    # ======================================================================

    def forward_to_edge(self, image, app, t_e2e_start, gpu_compute_ms):
        try:
            success, buffer = cv2.imencode(
                '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                return

            headers = {
                'Content-Type'     : 'application/octet-stream',
                'Frame-Width'      : str(self.frame_width),
                'Frame-Height'     : str(self.frame_height),
                'Client-Timestamp' : str(time.time()),
                'Frame-Identifier' : f"frame_{self.frame_id}",
                'detection-flag'   : app,
            }

            # ?? gpu_idle = network round-trip only ?????????????????????
            # FIX: t_net_start measured AFTER encode, so gpu_idle_ms is
            # purely the network wait, not encoding time.
            t_net_start = time.time()
            response    = requests.post(
                self.server_url,
                data=buffer.tobytes(),
                headers=headers,
                timeout=self.edge_timeout,
            )
            t_net_end = time.time()

            true_e2e_ms = (t_net_end - t_e2e_start) * 1000.0
            gpu_idle_ms = (t_net_end - t_net_start) * 1000.0   # pure network wait

            print(f"{Color.CYAN}[{app}] E2E:{true_e2e_ms:.0f}ms "
                  f"net:{gpu_idle_ms:.0f}ms "
                  f"gpu_compute:0ms{Color.RESET}")

            if response.status_code == 200:
                self.parse_and_log(
                    response, app, true_e2e_ms, gpu_compute_ms, gpu_idle_ms)
            else:
                rospy.logwarn(f"Edge {response.status_code} for {app}")

        except requests.exceptions.Timeout:
            rospy.logwarn(f"Timeout: {app}")
        except Exception as e:
            rospy.logerr(f"Edge error ({app}): {e}")

    def parse_and_log(self, response, app, true_e2e_ms,
                      gpu_compute_ms, gpu_idle_ms):
        try:
            result = response.json()
        except json.JSONDecodeError:
            return

        # ?? Extract detections by app key ?????????????????????????????
        detections = []
        if app == 'collision_avoidance':
            raw = result.get('OBD_model', {})
            if isinstance(raw, str):
                raw = json.loads(raw)
            detections = raw.get('detections', [])
        elif app == 'traffic_light_detection':
            raw = result.get('traffic_light_model',
                             result.get('detections', []))
            if isinstance(raw, str): raw = json.loads(raw)
            detections = raw.get('detections', [raw]) \
                         if isinstance(raw, dict) else raw
        elif app == 'traffic_sign_detection':
            raw = result.get('traffic_sign_model',
                             result.get('detections', []))
            if isinstance(raw, str): raw = json.loads(raw)
            detections = raw.get('detections', [raw]) \
                         if isinstance(raw, dict) else raw
        else:
            raw = result.get('detections', [])
            detections = raw if isinstance(raw, list) else []

        # ?? Unique labels ??????????????????????????????????????????????
        class_count = {}
        for det in detections:
            cls = det.get('class', 'unknown').lower()
            class_count[cls] = class_count.get(cls, 0) + 1
            det['unique_label'] = f"{cls}_{class_count[cls]}"

        # ?? Depth distance (collision only) ???????????????????????????
        if app == 'collision_avoidance':
            with self.lock:
                depth = self.latest_depth_image
            for det in detections:
                det['distance'] = self._compute_distance(
                    det.get('bbox'), depth)
            self.target_distance = None
            for det in detections:
                if det.get('unique_label') == TARGET_VEHICLE_LABEL:
                    self.target_distance = det.get('distance')
                    break
            dists = [d.get('distance') for d in detections
                     if d.get('distance') is not None and d.get('distance') > 0]
            self.min_distance = min(dists) if dists else None

        with self.lock:
            self.app_detections[app] = detections

        self.obj_pub.publish(String(data=json.dumps(detections)))

        metrics     = self.res_monitor.get()
        speed_kmh   = self._speed()
        det_classes = json.dumps([d.get('class', 'unknown') for d in detections])

        print(f"{Color.BOLD}{Color.GREEN}[{app}] "
              f"det={len(detections)} | e2e={true_e2e_ms:.0f}ms | "
              f"gpu_compute=0ms | gpu_idle(net)={gpu_idle_ms:.0f}ms"
              f"{Color.RESET}")

        self.async_csv.write_row([
            time.time(),
            self.frame_id,
            app,
            "edge",
            round(true_e2e_ms,    3),
            round(gpu_compute_ms, 3),          # always 0.0 ? no local GPU inference
            round(gpu_idle_ms,    3),           # pure network round-trip
            metrics['cpu'],
            metrics['system_gpu'],             # machine-wide GPU% (CARLA incl.) ? high is expected
            metrics['proc_gpu_mb'],            # this process only ? ~0 MB confirms edge uses no GPU
            metrics['ram'],
            metrics['tx_mbps'], metrics['rx_mbps'],
            round(speed_kmh, 2),
            self.target_distance if self.target_distance else "",
            self.min_distance    if self.min_distance    else "",
            len(detections),
            det_classes,
            1 if self.collision_detected else 0,
            self.collision_actor_id
                if self.collision_actor_id is not None else "",
        ])

        if app == self.applications[-1]:
            self.frame_id += 1

    # ======================================================================
    # HELPERS
    # ======================================================================

    def _compute_distance(self, bbox, depth_image):
        if bbox is None or depth_image is None:
            return -1.0
        try:
            x1, y1, x2, y2 = map(int, bbox)
            h, w = depth_image.shape
            x1 = max(0, min(x1, w-1)); x2 = max(0, min(x2, w-1))
            y1 = max(0, min(y1, h-1)); y2 = max(0, min(y2, h-1))
            if x2 > x1 and y2 > y1:
                roi = depth_image[y1:y2, x1:x2]
                if roi.size > 0:
                    return float(np.median(roi))
        except:
            pass
        return -1.0

    def _speed(self):
        if self.ego_odom is None:
            return 0.0
        v = self.ego_odom.twist.twist.linear
        return math.sqrt(v.x**2 + v.y**2) * 3.6

    # ======================================================================
    # VEHICLE CONTROL
    # ======================================================================

    def control_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.control_vehicle()
            except Exception as e:
                rospy.logerr(f"[CONTROL] {e}")
            rate.sleep()

    def control_vehicle(self):
        ctrl = CarlaEgoVehicleControl()
        ctrl.steer = 0.0; ctrl.hand_brake = False; ctrl.reverse = False
        speed = self._speed()

        if not self.ego_moving:
            ctrl.throttle = 0.0; ctrl.brake = 1.0; status = "STOPPED"
        elif (self.target_distance is not None and
              self.target_distance <= SAFE_DISTANCE_M):
            ctrl.throttle = 0.0; ctrl.brake = 1.0; status = "EMERGENCY BRAKE"
        elif speed < TARGET_SPEED_KMH:
            ctrl.throttle = MAX_THROTTLE; ctrl.brake = 0.0
            status = "ACCELERATING"
        else:
            ctrl.throttle = 0.0; ctrl.brake = 0.0; status = "CRUISING"

        self.control_pub.publish(ctrl)
        if not hasattr(self, '_lcp') or time.time() - self._lcp > 0.5:
            print(f"[CONTROL] {status} | {speed:.1f}km/h | "
                  f"dist:{self.target_distance}")
            self._lcp = time.time()

    def keyboard_control(self):
        def on_press(key):
            try:
                if hasattr(key, 'char'):
                    if key.char == 's':
                        self.ego_moving = True
                        print(f"\n{Color.GREEN}[KEY] ? START{Color.RESET}\n")
                    elif key.char == 'a':
                        self.ego_moving = False
                        print(f"\n{Color.RED}[KEY] ? STOP{Color.RESET}\n")
            except AttributeError:
                pass
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        listener.join()

    def shutdown(self):
        print(f"\n{Color.RED}[SHUTDOWN]{Color.RESET}")
        ctrl = CarlaEgoVehicleControl()
        ctrl.throttle = 0.0; ctrl.brake = 1.0
        self.control_pub.publish(ctrl)
        self.res_monitor.shutdown()
        self.async_csv.close()
        cv2.destroyAllWindows()
        print(f"{Color.GREEN}[SHUTDOWN] ?{Color.RESET}\n")


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    node = None
    try:
        node = EdgeForwarderOptimized()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
