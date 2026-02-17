#!/usr/bin/env python3
"""
Optimized Pure Edge Offload System - TRUE E2E Latency + GPU Tracking
=====================================================================
Measures ONLY edge send/receive time as TRUE E2E latency.
Now includes GPU compute/idle time tracking (GPU idle = network wait time).

Author: Vidya Vepoori
Date: 2026-02-06
Version: OPTIMIZED_WITH_RESOURCES + GPU TRACKING
"""

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
import torch

# ============================================================================
# CONFIGURATION
# ============================================================================

CSV_FILE = "/media/root/b9f388cb-81d8-4121-93fb-ad514e324552/vidyav/main/edge_plot_gpu_tracking_1.csv"
INTERFACE = "enp25s0f0"
GPU_INDEX = 0
ARTIFICIAL_DELAY_MS = 0

# Vehicle control parameters
TARGET_SPEED_KMH = 30
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
# RESOURCE MONITOR CLASS (NON-BLOCKING)
# ============================================================================

class ResourceMonitor:
    """Background resource monitor - doesn't block critical path."""
    
    def __init__(self, gpu_index=0, interfaces=None):
        self.interfaces = interfaces or []
        nvmlInit()
        self.gpu_handle = nvmlDeviceGetHandleByIndex(gpu_index)
        self.prev_net = psutil.net_io_counters(pernic=True)
        self.prev_time = time.time()
        
        # Cache latest values (updated in background)
        self.latest_cpu = 0.0
        self.latest_gpu = 0
        self.latest_ram = 0.0
        self.latest_tx_mbps = 0.0
        self.latest_rx_mbps = 0.0
        
        # Start background monitoring thread
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

    def _monitor_loop(self):
        """Background thread that updates resource metrics every 100ms."""
        while self.monitoring:
            try:
                # Update CPU/GPU/RAM
                self.latest_cpu = psutil.cpu_percent(interval=None)
                self.latest_ram = psutil.virtual_memory().percent
                self.latest_gpu = nvmlDeviceGetUtilizationRates(self.gpu_handle).gpu
                
                # Update network bandwidth
                now = time.time()
                interval = now - self.prev_time
                net = psutil.net_io_counters(pernic=True)
                
                iface = self.interfaces[0] if self.interfaces else None
                if iface and iface in net and iface in self.prev_net and interval > 0:
                    tx = (net[iface].bytes_sent - self.prev_net[iface].bytes_sent) * 8 / (1e6 * interval)
                    rx = (net[iface].bytes_recv - self.prev_net[iface].bytes_recv) * 8 / (1e6 * interval)
                    self.latest_tx_mbps = tx
                    self.latest_rx_mbps = rx
                
                self.prev_net = net
                self.prev_time = now
                
            except Exception as e:
                rospy.logwarn(f"Resource monitor error: {e}")
            
            time.sleep(0.1)  # Update every 100ms

    def get_latest_metrics(self):
        """Get cached metrics without blocking."""
        return {
            'cpu': self.latest_cpu,
            'gpu': self.latest_gpu,
            'ram': self.latest_ram,
            'tx_mbps': self.latest_tx_mbps,
            'rx_mbps': self.latest_rx_mbps
        }

    def shutdown(self):
        """Stop monitoring and cleanup."""
        self.monitoring = False
        nvmlShutdown()


# ============================================================================
# MAIN EDGE FORWARDER - OPTIMIZED VERSION + GPU TRACKING
# ============================================================================

class EdgeForwarderOptimized:
    """
    Optimized edge offload system with GPU compute/idle tracking.
    GPU Compute Time = minimal (only CARLA rendering on local GPU)
    GPU Idle Time = network wait time (waiting for edge response)
    """
    
    def __init__(self):
        rospy.init_node('edge_forwarder_optimized', anonymous=True)
        
        print(f"\n{Color.BOLD}{Color.CYAN}{'='*80}")
        print(f"  OPTIMIZED EDGE OFFLOAD SYSTEM + GPU TRACKING")
        print(f"  PURE E2E LATENCY MEASUREMENT (Send/Receive Only)")
        print(f"  GPU Compute/Idle Time Tracking")
        print(f"{'='*80}{Color.RESET}\n")
        
        # -------- ROS PARAMETERS --------
        self.server_url = rospy.get_param('~server_url')
        self.edge_timeout = rospy.get_param('~edge_timeout', 6.0)
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.image_topic = rospy.get_param('~image_topic')
        self.depth_topic = rospy.get_param('~depth_topic')
        self.applications = rospy.get_param('~applications', [
            'lane_detection', 'collision_avoidance'])

        # -------- STATE VARIABLES --------
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame_id = 0
        self.detection_results = []
        self.latest_depth_image = None
        
        # Vehicle state
        self.ego_odom = None
        self.ego_moving = False
        self.collision_detected = False
        self.collision_actor_id = None
        self.target_distance = None
        self.min_distance = None

        # -------- ROS SUBSCRIBERS --------
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=10
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
        self.obj_pub = rospy.Publisher('/carla/edge_detection', String, queue_size=1)
        self.control_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd',
            CarlaEgoVehicleControl,
            queue_size=1
        )

        # -------- RESOURCE MONITOR (BACKGROUND) --------
        print(f"{Color.YELLOW}[INIT] Starting background resource monitor...{Color.RESET}")
        self.res_monitor = ResourceMonitor(GPU_INDEX, [INTERFACE])
        print(f"{Color.GREEN}[INIT] ? Resource monitor running in background{Color.RESET}\n")

        # -------- CSV LOGGING SETUP --------
        print(f"{Color.YELLOW}[INIT] Setting up CSV logger: {CSV_FILE}{Color.RESET}")
        self.csv_file = open(CSV_FILE, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write CSV header - includes GPU compute/idle fields
        self.csv_writer.writerow([
            "timestamp",
            "frame_id",
            "app_type",
            "true_e2e_ms",
            "gpu_compute_ms",
            "gpu_idle_ms",
            "cpu",
            "gpu",
            "ram",
            "tx_mbps",
            "rx_mbps",
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

        rospy.loginfo(f"EdgeForwarderOptimized subscribed to {self.image_topic}, forwarding to {self.server_url}")

    # ========================================================================
    # ROS CALLBACKS
    # ========================================================================

    def image_callback(self, msg):
        """Main image callback - triggers pure edge offload measurement with GPU tracking."""
        print(f"\n{Color.CYAN}{'?'*70}")
        print(f"[FRAME {self.frame_id}] Received - Starting PURE E2E + GPU tracking")
        print(f"{'?'*70}{Color.RESET}")
        
        try:
            # ============================================================
            # START TRUE E2E TIMER - Pure send/receive measurement
            # ============================================================
            t_e2e_start = time.time()
            
            # Synchronize GPU (CARLA rendering on local GPU)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
                t_gpu_compute_start = time.time()
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # GPU compute time = minimal (just image conversion/encoding)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
                gpu_compute_ms = (time.time() - t_gpu_compute_start) * 1000.0
            else:
                gpu_compute_ms = 0.0
            
            # Process each application
            for app in self.applications:
                self.forward_to_edge_pure(cv_image.copy(), app, t_e2e_start, gpu_compute_ms)
                
        except Exception as e:
            rospy.logwarn(f"Image callback error: {e}")

    def depth_callback(self, msg):
        """Depth image callback."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth_image.dtype != np.float32:
                depth_image = depth_image.astype(np.float32)
            with self.lock:
                self.latest_depth_image = depth_image
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
    # OPTIMIZED EDGE OFFLOAD - PURE E2E MEASUREMENT + GPU TRACKING
    # ========================================================================

    def forward_to_edge_pure(self, frame, app, t_e2e_start, gpu_compute_ms):
        """
        Pure edge offload with GPU tracking.
        GPU Compute = minimal local processing
        GPU Idle = network wait time (waiting for edge server response)
        """
        try:
            # Resize frame
            image = cv2.resize(frame, (self.frame_width, self.frame_height))
            
            # Apply artificial delay if configured
            if ARTIFICIAL_DELAY_MS > 0:
                time.sleep(ARTIFICIAL_DELAY_MS / 1000.0)
            
            # Encode image
            success, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                rospy.logwarn("Failed to encode image")
                return

            # Prepare headers
            headers = {
                'Content-Type': 'application/octet-stream',
                'Frame-Width': str(self.frame_width),
                'Frame-Height': str(self.frame_height),
                'Client-Timestamp': str(time.time()),
                'Frame-Identifier': f"frame_{self.frame_id}",
                'detection-flag': app
            }

            print(f"{Color.CYAN}[EDGE] Sending frame {self.frame_id} to edge server...{Color.RESET}")
            
            # Mark start of network wait (GPU idle period)
            t_network_wait_start = time.time()
            
            # Send to edge server (GPU is IDLE during this time)
            response = requests.post(
                self.server_url,
                data=buffer.tobytes(),
                headers=headers,
                timeout=self.edge_timeout
            )
            
            # ============================================================
            # END TRUE E2E TIMER - Response received
            # ============================================================
            t_e2e_end = time.time()
            true_e2e_ms = (t_e2e_end - t_e2e_start) * 1000.0
            
            # Calculate GPU idle time (network wait)
            gpu_idle_ms = (t_e2e_end - t_network_wait_start) * 1000.0
            
            print(f"{Color.CYAN}[EDGE] Response received | Status: {response.status_code}{Color.RESET}")
            print(f"{Color.BLUE}[GPU] Compute: {gpu_compute_ms:.2f}ms | Idle (Network Wait): {gpu_idle_ms:.2f}ms{Color.RESET}")

            if response.status_code == 200:
                # Process response
                self.process_edge_response_pure(
                    response, image, app, true_e2e_ms, gpu_compute_ms, gpu_idle_ms
                )
            else:
                rospy.logwarn(f"Edge server returned status {response.status_code} for {app}")

        except requests.exceptions.Timeout:
            rospy.logwarn(f"Edge request timed out for {app}")
        except requests.exceptions.RequestException as e:
            rospy.logwarn(f"Edge request failed for {app}: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in forward_to_edge_pure ({app}): {e}")

    def process_edge_response_pure(self, response, frame, app, true_e2e_ms, gpu_compute_ms, gpu_idle_ms):
        """
        Process edge server response and log pure E2E latency with GPU tracking.
        """
        try:
            result = response.json()
        except json.JSONDecodeError:
            rospy.logwarn("Edge response not valid JSON")
            return

        with self.lock:
            # Parse detections
            if 'OBD_model' in result:
                obj_det = result['OBD_model']
                if isinstance(obj_det, str):
                    try:
                        obj_det = json.loads(obj_det)
                    except:
                        obj_det = {}
                self.detection_results = obj_det.get('detections', [])

                # Assign unique IDs
                class_count = {}
                for det in self.detection_results:
                    cls = det.get("class", "unknown").lower()
                    class_count[cls] = class_count.get(cls, 0) + 1
                    det['unique_label'] = f"{cls}_{class_count[cls]}"

                # Compute distance if depth available
                for det in self.detection_results:
                    bbox = det.get("bbox")
                    if bbox and self.latest_depth_image is not None:
                        x1, y1, x2, y2 = map(int, bbox)
                        x1 = max(0, min(x1, self.latest_depth_image.shape[1]-1))
                        x2 = max(0, min(x2, self.latest_depth_image.shape[1]-1))
                        y1 = max(0, min(y1, self.latest_depth_image.shape[0]-1))
                        y2 = max(0, min(y2, self.latest_depth_image.shape[0]-1))
                        
                        if x2 > x1 and y2 > y1:
                            roi = self.latest_depth_image[y1:y2, x1:x2]
                            if roi.size > 0:
                                det['distance'] = float(np.median(roi))
                            else:
                                det['distance'] = -1.0
                        else:
                            det['distance'] = -1.0
                    else:
                        det['distance'] = -1.0

                # Find target distance (car_1) and min distance
                self.target_distance = None
                for det in self.detection_results:
                    if det.get('unique_label') == TARGET_VEHICLE_LABEL:
                        self.target_distance = det.get('distance')
                        if self.target_distance == -1.0:
                            self.target_distance = None
                        break
                
                # Find minimum distance
                distances = [d.get('distance') for d in self.detection_results 
                           if d.get('distance') is not None and d.get('distance') > 0]
                self.min_distance = min(distances) if distances else None

                # Draw and display
                self.draw_objects(frame)
                cv2.imshow("Edge Detection View - Optimized", frame)
                cv2.waitKey(1)

                # Get vehicle speed
                speed_kmh = self.get_speed_from_odom()
                
                # Get number of detections
                num_detections = len(self.detection_results)

                # ============================================================
                # Get cached resource metrics (NO TIMING IMPACT)
                # ============================================================
                metrics = self.res_monitor.get_latest_metrics()
                cpu = metrics['cpu']
                gpu = metrics['gpu']
                ram = metrics['ram']
                tx_mbps = metrics['tx_mbps']
                rx_mbps = metrics['rx_mbps']

                # ============================================================
                # Print PURE E2E latency + GPU tracking + system info
                # ============================================================
                print(f"\n{Color.BOLD}{Color.GREEN}?????????????????????????????????????????????")
                print(f"?  PURE E2E LATENCY + GPU TRACKING - Frame {self.frame_id}")
                print(f"?????????????????????????????????????????????")
                print(f"?")
                print(f"?  TRUE E2E LATENCY:   {true_e2e_ms:>8.2f} ms          ?")
                print(f"?")
                print(f"?  {Color.BLUE}GPU Breakdown:{Color.RESET}")
                print(f"?    ?? GPU Compute:   {gpu_compute_ms:>8.2f} ms  ?")
                print(f"?    ?? GPU Idle:      {gpu_idle_ms:>8.2f} ms  ?")
                print(f"?")
                print(f"?  {Color.BLUE}System Resources (Background):{Color.RESET}")
                print(f"?    CPU: {cpu:>5.1f}% | GPU: {gpu:>3}% | RAM: {ram:>5.1f}%")
                print(f"?    TX:  {tx_mbps:>7.2f} Mbps | RX: {rx_mbps:>7.2f} Mbps")
                print(f"?")
                print(f"?  {Color.YELLOW}Detection Results:{Color.RESET}")
                print(f"?    Detections:      {num_detections} objects")
                print(f"?    Target Distance: {self.target_distance if self.target_distance else 'N/A'}")
                print(f"?    Min Distance:    {self.min_distance if self.min_distance else 'N/A'}")
                print(f"?    Speed:           {speed_kmh:.1f} km/h")
                print(f"?    Collision:       {'YES' if self.collision_detected else 'NO'}")
                print(f"?")
                print(f"?????????????????????????????????????????????{Color.RESET}")

                # ============================================================
                # Log to CSV - includes GPU compute/idle
                # ============================================================
                self.csv_writer.writerow([
                    time.time(),                                                      # timestamp
                    self.frame_id,                                                    # frame_id
                    app,                                                              # app_type
                    true_e2e_ms,                                                      # true_e2e_ms (PURE)
                    gpu_compute_ms,                                                   # gpu_compute_ms ?
                    gpu_idle_ms,                                                      # gpu_idle_ms ?
                    cpu,                                                              # cpu (background)
                    gpu,                                                              # gpu (background)
                    ram,                                                              # ram (background)
                    tx_mbps,                                                          # tx_mbps (background)
                    rx_mbps,                                                          # rx_mbps (background)
                    speed_kmh,                                                        # speed_kmh
                    self.target_distance if self.target_distance is not None else "", # target_dist
                    self.min_distance if self.min_distance is not None else "",       # min_dist
                    num_detections,                                                   # num_detections
                    1 if self.collision_detected else 0,                             # collision
                    self.collision_actor_id if self.collision_actor_id is not None else ""  # collision_id
                ])
                self.csv_file.flush()

                # Publish detections
                self.obj_pub.publish(String(data=json.dumps(self.detection_results)))
                
                # Increment frame ID
                self.frame_id += 1

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
        current_speed = self.get_speed_from_odom()
        
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

    def get_speed_from_odom(self):
        """Calculate speed in km/h from odometry message."""
        if self.ego_odom is None:
            return 0.0
        vel = self.ego_odom.twist.twist.linear
        vx = vel.x
        vy = vel.y
        return math.sqrt(vx*vx + vy*vy) * 3.6  # Convert m/s to km/h

    # ========================================================================
    # DISPLAY
    # ========================================================================

    def draw_objects(self, frame):
        """Draw bounding boxes and labels on frame."""
        for det in self.detection_results:
            obj_class = det.get("class", "").lower()
            if obj_class not in ['car', 'pedestrian']:
                continue
            bbox = det.get('bbox')
            if bbox and all(v is not None for v in bbox):
                x1, y1, x2, y2 = map(int, bbox)
                
                # Color based on distance
                distance = det.get('distance', -1.0)
                if obj_class == 'car':
                    if distance > 0 and distance <= SAFE_DISTANCE_M:
                        color = (0, 0, 255)  # Red - danger
                    else:
                        color = (255, 0, 0)  # Blue - safe
                else:
                    color = (0, 255, 0)  # Green - pedestrian
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, det.get('unique_label', obj_class), (x1, y1-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                if distance > 0:
                    cv2.putText(frame, f"{distance:.2f}m", (x1, y2+15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # ========================================================================
    # SHUTDOWN
    # ========================================================================

    def shutdown(self):
        """Clean shutdown handler."""
        print(f"\n{Color.RED}[SHUTDOWN] Shutting down Optimized Edge Forwarder...{Color.RESET}")
        
        # Stop vehicle
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        self.control_pub.publish(control_msg)
        print(f"{Color.GREEN}[SHUTDOWN] ? Vehicle stopped{Color.RESET}")
        
        # Close resources
        self.res_monitor.shutdown()
        self.csv_file.close()
        cv2.destroyAllWindows()
        
        print(f"{Color.GREEN}[SHUTDOWN] ? System shutdown complete{Color.RESET}\n")


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    node = None
    try:
        node = EdgeForwarderOptimized()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Optimized edge forwarder node terminated")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        if node is not None:
            node.shutdown()
