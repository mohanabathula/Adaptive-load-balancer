"""
Real-Time Traffic Signal Detection System
IMPROVED VERSION: Two-stage detection
1. First detects traffic light structure (vertical circles)
2. Then identifies which light is active
This prevents false positives from random red/yellow/green objects
"""

import cv2
import numpy as np
from collections import deque
import time


class TrafficSignalDetector:
    def __init__(self):
        """Initialize the traffic signal detector"""
        self.signal_states = {
            'red': 0,
            'yellow': 0,
            'green': 0
        }
        
        # Color ranges in HSV - MORE RESTRICTIVE for actual traffic lights
        # Higher saturation and value thresholds = only bright, vivid colors
        self.color_ranges = {
            'red': [
                (np.array([0, 150, 150]), np.array([10, 255, 255])),  # Brighter reds
                (np.array([170, 150, 150]), np.array([180, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 150, 150]), np.array([35, 255, 255]))  # Brighter yellows
            ],
            'green': [
                (np.array([45, 100, 100]), np.array([85, 255, 255]))  # Brighter greens
            ]
        }
        
        # Frame buffer for stable detection
        self.detection_buffer = deque(maxlen=7)  # Increased for more stability
        self.current_signal = "No Signal"
        
    def detect_circles(self, frame):
        """Detect circular shapes that could be traffic lights"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        circles = cv2.HoughCircles(
            gray_blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=30,  # Reduced - traffic lights have multiple circles close together
            param1=100,  # Increased - more selective edge detection
            param2=30,
            minRadius=8,
            maxRadius=50
        )
        
        return circles
    
    def group_vertical_circles(self, circles):
        """
        STAGE 1: Group circles that are vertically aligned (traffic light pattern)
        This filters out random circular objects
        """
        if circles is None:
            return []
        
        circles = np.uint16(np.around(circles[0]))
        traffic_light_groups = []
        used = set()
        
        for i, (x1, y1, r1) in enumerate(circles):
            if i in used:
                continue
            
            group = [(x1, y1, r1)]
            group_indices = [i]
            
            # Find vertically aligned circles
            for j, (x2, y2, r2) in enumerate(circles):
                if i == j or j in used:
                    continue
                
                # Check vertical alignment - convert to int to avoid overflow
                h_dist = abs(int(x1) - int(x2))
                v_dist = abs(int(y1) - int(y2))
                
                # Traffic lights: small horizontal offset, reasonable vertical spacing
                if h_dist < 40 and 20 < v_dist < 150:
                    # Radii should be similar
                    radius_diff = abs(int(r1) - int(r2)) / max(int(r1), int(r2))
                    if radius_diff < 0.5:
                        group.append((x2, y2, r2))
                        group_indices.append(j)
            
            # Need at least 2 circles for a traffic light
            if len(group) >= 2:
                for idx in group_indices:
                    used.add(idx)
                traffic_light_groups.append(sorted(group, key=lambda c: c[1]))  # Sort top to bottom
        
        return traffic_light_groups
    
    def detect_color_in_region(self, frame, x, y, radius):
        """
        STAGE 2: Detect which color is present in a circular region
        Only checks if light is BRIGHT and ON
        """
        # Create mask for the circular region
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (x, y), radius, 255, -1)
        
        # Extract the region
        roi = cv2.bitwise_and(frame, frame, mask=mask)
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Also check brightness
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked_gray = cv2.bitwise_and(gray, mask)
        avg_brightness = np.mean(masked_gray[masked_gray > 0]) if np.any(masked_gray > 0) else 0
        
        # Only proceed if the area is bright enough (light is ON)
        if avg_brightness < 120:  # Threshold for brightness
            return None
        
        detected_colors = {}
        
        for color_name, ranges in self.color_ranges.items():
            color_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            
            for lower, upper in ranges:
                temp_mask = cv2.inRange(hsv_roi, lower, upper)
                color_mask = cv2.bitwise_or(color_mask, temp_mask)
            
            # Apply the circular mask
            color_mask = cv2.bitwise_and(color_mask, mask)
            
            # Count non-zero pixels and weight by brightness
            pixel_count = cv2.countNonZero(color_mask)
            score = pixel_count * (avg_brightness / 255.0)
            detected_colors[color_name] = score
        
        # Return the color with maximum score if above threshold
        max_color = max(detected_colors, key=detected_colors.get)
        if detected_colors[max_color] > 200:  # Higher threshold - must be bright AND colored
            return max_color
        
        return None
    
    def detect_traffic_lights(self, frame):
        """
        Main detection function - TWO STAGE PROCESS:
        1. Find traffic light structures (vertical circle groups)
        2. Detect which light is active (bright + colored)
        """
        detected_lights = []
        
        circles = self.detect_circles(frame)
        
        # STAGE 1: Group circles into traffic light structures
        traffic_light_groups = self.group_vertical_circles(circles)
        
        if not traffic_light_groups:
            return []
        
        # STAGE 2: For each traffic light structure, find the active light
        for group in traffic_light_groups:
            active_light = None
            max_score = 0
            
            for x, y, radius in group:
                color = self.detect_color_in_region(frame, x, y, radius)
                
                if color:
                    # Calculate confidence score
                    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                    cv2.circle(mask, (x, y), radius, 255, -1)
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    brightness = np.mean(gray[mask > 0])
                    
                    if brightness > max_score:
                        max_score = brightness
                        active_light = {
                            'position': (x, y),
                            'radius': radius,
                            'color': color,
                            'confidence': brightness,
                            'group_size': len(group)
                        }
            
            # Only add if we found an active light
            if active_light:
                detected_lights.append(active_light)
        
        return detected_lights
    
    def draw_detections(self, frame, detections):
        """Draw bounding boxes and labels on detected traffic lights"""
        for detection in detections:
            x, y = detection['position']
            radius = detection['radius']
            color = detection['color']
            confidence = detection.get('confidence', 0)
            group_size = detection.get('group_size', 1)
            
            # Color mapping for visualization
            color_map = {
                'red': (0, 0, 255),
                'yellow': (0, 255, 255),
                'green': (0, 255, 0)
            }
            
            # Draw circle around detected light (thicker for better visibility)
            cv2.circle(frame, (x, y), radius + 5, color_map[color], 3)
            cv2.circle(frame, (x, y), radius, color_map[color], 2)
            
            # Draw bounding box for the traffic light structure
            box_size = radius * 4
            cv2.rectangle(frame, 
                         (x - box_size, y - box_size), 
                         (x + box_size, y + box_size), 
                         (255, 255, 255), 2)
            
            # Draw label with background
            label = f"{color.upper()}"
            conf_text = f"({confidence:.0f})"
            
            # Text background for better visibility
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
            cv2.rectangle(frame, 
                         (x - radius, y - radius - 40), 
                         (x - radius + text_size[0] + 10, y - radius - 10), 
                         color_map[color], -1)
            
            cv2.putText(frame, label, (x - radius + 5, y - radius - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            # Show number of lights in structure
            cv2.putText(frame, f"{group_size} lights", (x - radius, y + radius + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Update detection buffer
            self.detection_buffer.append(color)
        
        # Determine stable signal state
        if len(self.detection_buffer) > 0:
            self.current_signal = max(set(self.detection_buffer), 
                                     key=self.detection_buffer.count)
        else:
            self.current_signal = "No Signal"
        
        return frame
    
    def add_info_overlay(self, frame, fps):
        """Add information overlay to the frame"""
        h, w = frame.shape[:2]
        
        # Create semi-transparent overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (380, 140), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.4, frame, 0.6, 0)
        
        # Color for current signal
        signal_colors = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'green': (0, 255, 0),
            'No Signal': (128, 128, 128)
        }
        signal_color = signal_colors.get(self.current_signal, (255, 255, 255))
        
        # Add text information
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Signal State:", (20, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"{self.current_signal.upper()}", 
                   (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, signal_color, 2)
        cv2.putText(frame, "Press 'q' to quit", (20, 125),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return frame
    
    def run(self, video_source=0):
        """Run the real-time detection system"""
        cap = cv2.VideoCapture(video_source)
        
        if not cap.isOpened():
            print(f"Error: Could not open video source {video_source}")
            print("\n🔧 TROUBLESHOOTING:")
            print("1. Check camera permissions: System Settings > Privacy & Security > Camera")
            print("2. Enable Terminal access to camera")
            print("3. Try different camera index: detector.run(video_source=1)")
            print("4. Ensure no other app is using the camera")
            return
        
        print("\n" + "="*70)
        print("🚦 IMPROVED TRAFFIC SIGNAL DETECTION SYSTEM 🚦")
        print("="*70)
        print("\n✨ KEY IMPROVEMENTS:")
        print("  1. Two-stage detection: Structure → Active Light")
        print("  2. Filters out random red/yellow/green objects")
        print("  3. Only detects actual traffic light patterns (2+ vertical circles)")
        print("  4. Checks brightness to ensure light is ON")
        print("  5. More restrictive color thresholds")
        print("\n📋 HOW IT WORKS:")
        print("  Stage 1: Finds vertically aligned circles (traffic light structure)")
        print("  Stage 2: Identifies which light is bright and colored (active)")
        print("\n🎮 CONTROLS:")
        print("  Press 'q' to quit")
        print("="*70 + "\n")
        
        # FPS calculation
        fps_counter = deque(maxlen=30)
        frame_count = 0
        
        while True:
            start_time = time.time()
            
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame")
                break
            
            frame_count += 1
            
            # Resize frame for better performance
            frame = cv2.resize(frame, (800, 600))
            
            # Detect traffic lights
            detections = self.detect_traffic_lights(frame)
            
            # Draw detections
            frame = self.draw_detections(frame, detections)
            
            # Calculate FPS
            fps = 1.0 / (time.time() - start_time)
            fps_counter.append(fps)
            avg_fps = np.mean(fps_counter)
            
            # Add information overlay
            frame = self.add_info_overlay(frame, avg_fps)
            
            # Add detection count
            if detections:
                cv2.putText(frame, f"Detected: {len(detections)} traffic light(s)", 
                           (10, 570), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Display the frame
            cv2.imshow('Traffic Signal Detection - Improved', frame)
            
            # Check for quit command
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        print("\n✅ Detection stopped.")
        print(f"📊 Processed {frame_count} frames")
        print("Thank you for using Traffic Signal Detection!\n")


def main():
    """Main function to run the detector"""
    detector = TrafficSignalDetector()
    
    # You can change the video source:
    # 0 for webcam
    # Or provide a video file path like 'traffic_video.mp4'
    detector.run(video_source=0)


if __name__ == "__main__":
    main()
