"""
Advanced Hierarchical Lane Detection for Indian Roads
Pipeline: Frame → Road → Lanes → Curvature → Output
Optimized for real-time with <50ms latency
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
import time


@dataclass
class RoadAnalysis:
    """Road detection result"""
    mask: np.ndarray
    quality: float  # 0-1 road surface quality
    type: str  # highway, city, rural


@dataclass
class LaneAnalysis:
    """Lane detection result"""
    left: Optional[np.ndarray]
    right: Optional[np.ndarray]
    center: Optional[np.ndarray]
    left_curve: float  # radius in meters
    right_curve: float
    deviation: float  # lateral offset from center
    confidence: float


@dataclass
class DetectionResult:
    """Complete hierarchical result"""
    road: RoadAnalysis
    lanes: LaneAnalysis
    fps: float
    latency_ms: float


class HierarchicalLaneDetector:
    """Advanced detector with hierarchical pipeline"""
    
    def __init__(self, width=640, height=480):
        self.w, self.h = width, height
        
        # Temporal smoothing buffers
        self.prev = {'left': None, 'right': None, 'center': None}
        self.alpha = 0.3  # smoothing factor
        
        # Calibration (pixels per meter)
        self.ym_per_pix = 30/720  # vertical
        self.xm_per_pix = 3.7/700  # horizontal (lane width ~3.7m)
    
    def detect(self, frame: np.ndarray) -> DetectionResult:
        """Main hierarchical detection pipeline"""
        start = time.time()
        
        # Resize and preprocess
        img = cv2.resize(frame, (self.w, self.h))
        
        # LEVEL 1: Road Detection
        road = self._detect_road(img)
        
        # LEVEL 2: Lane Detection (within road area)
        lanes = self._detect_lanes(img, road.mask)
        
        # LEVEL 3: Curvature Analysis
        lanes = self._analyze_curvature(lanes)
        
        latency = (time.time() - start) * 1000
        fps = 1000 / latency
        
        return DetectionResult(road, lanes, fps, latency)
    
    def _detect_road(self, img: np.ndarray) -> RoadAnalysis:
        """LEVEL 1: Detect and analyze road surface"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Road color range (Indian roads: asphalt, concrete, red-brown)
        mask1 = cv2.inRange(hsv, (0, 0, 40), (180, 50, 220))  # General road
        mask2 = cv2.inRange(hsv, (0, 40, 60), (25, 255, 180))  # Red-brown roads
        road_mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply trapezoid ROI
        roi_mask = np.zeros_like(road_mask)
        pts = np.array([[
            [int(self.w*0.1), self.h],
            [int(self.w*0.45), int(self.h*0.55)],
            [int(self.w*0.55), int(self.h*0.55)],
            [int(self.w*0.9), self.h]
        ]], dtype=np.int32)
        cv2.fillPoly(roi_mask, pts, 255)
        road_mask = cv2.bitwise_and(road_mask, roi_mask)
        
        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)
        
        # Analyze road quality and type
        quality = np.sum(road_mask > 0) / np.sum(roi_mask > 0)
        road_type = 'highway' if quality > 0.7 else 'city' if quality > 0.5 else 'rural'
        
        return RoadAnalysis(road_mask, quality, road_type)
    
    def _detect_lanes(self, img: np.ndarray, road_mask: np.ndarray) -> LaneAnalysis:
        """LEVEL 2: Detect lanes within road area"""
        # Enhanced preprocessing
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # CLAHE for better contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        
        # Multi-channel lane detection
        # 1. White lanes (grayscale threshold)
        _, white = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        
        # 2. Yellow lanes (HSV)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, (15, 80, 150), (35, 255, 255))
        
        # Combine lane masks
        lane_mask = cv2.bitwise_or(white, yellow)
        lane_mask = cv2.bitwise_and(lane_mask, road_mask)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        edges = cv2.bitwise_and(edges, road_mask)
        
        # Combine color and edge detection
        combined = cv2.bitwise_or(lane_mask, edges)
        
        # Hough line detection
        lines = cv2.HoughLinesP(combined, 1, np.pi/180, 30, 
                                minLineLength=40, maxLineGap=100)
        
        # Classify and fit lanes
        left, right, center = self._classify_and_fit_lanes(lines)
        
        # Calculate confidence
        conf = self._calculate_confidence(left, right, lines)
        
        # Calculate deviation
        deviation = self._calculate_deviation(left, right)
        
        return LaneAnalysis(left, right, center, 0.0, 0.0, deviation, conf)
    
    def _classify_and_fit_lanes(self, lines) -> Tuple:
        """Classify lines into left/right/center and fit curves"""
        if lines is None:
            return self.prev['left'], self.prev['right'], self.prev['center']
        
        left_pts, right_pts, center_pts = [], [], []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            
            slope = (y2 - y1) / (x2 - x1)
            angle = abs(np.degrees(np.arctan(slope)))
            
            # Filter by angle (20-85 degrees for valid lanes)
            if angle < 20 or angle > 85:
                continue
            
            mid_x = (x1 + x2) / 2
            
            # Classify by position and slope
            if abs(mid_x - self.w/2) < self.w*0.15:  # Center lane
                center_pts.extend([[x1, y1], [x2, y2]])
            elif slope < 0 and mid_x < self.w*0.6:  # Left lane
                left_pts.extend([[x1, y1], [x2, y2]])
            elif slope > 0 and mid_x > self.w*0.4:  # Right lane
                right_pts.extend([[x1, y1], [x2, y2]])
        
        # Fit polynomial curves with smoothing
        left = self._fit_smooth(left_pts, 'left')
        right = self._fit_smooth(right_pts, 'right')
        center = self._fit_smooth(center_pts, 'center')
        
        return left, right, center
    
    def _fit_smooth(self, pts, side: str):
        """Fit polynomial with temporal smoothing"""
        if len(pts) < 4:
            return self.prev[side]
        
        pts = np.array(pts)
        x, y = pts[:, 0], pts[:, 1]
        
        try:
            # Fit 2nd degree polynomial
            z = np.polyfit(y, x, 2)
            y_range = np.linspace(self.h*0.55, self.h, 40)
            x_fit = np.polyval(z, y_range)
            curve = np.column_stack((x_fit, y_range)).astype(np.int32)
            
            # Temporal smoothing
            if self.prev[side] is not None:
                curve = (self.alpha * curve + (1-self.alpha) * self.prev[side]).astype(np.int32)
            
            self.prev[side] = curve
            return curve
        except:
            return self.prev[side]
    
    def _analyze_curvature(self, lanes: LaneAnalysis) -> LaneAnalysis:
        """LEVEL 3: Calculate curvature radius"""
        lanes.left_curve = self._calc_curve_radius(lanes.left)
        lanes.right_curve = self._calc_curve_radius(lanes.right)
        return lanes
    
    def _calc_curve_radius(self, lane) -> float:
        """Calculate radius of curvature in meters"""
        if lane is None or len(lane) < 3:
            return float('inf')
        
        # Convert to world space
        y = lane[:, 1] * self.ym_per_pix
        x = lane[:, 0] * self.xm_per_pix
        
        try:
            # Fit polynomial in world space
            fit = np.polyfit(y, x, 2)
            y_eval = np.max(y)
            
            # Calculate radius
            radius = ((1 + (2*fit[0]*y_eval + fit[1])**2)**1.5) / abs(2*fit[0])
            return radius if radius < 10000 else float('inf')
        except:
            return float('inf')
    
    def _calculate_deviation(self, left, right) -> float:
        """Calculate lateral deviation from lane center"""
        if left is None or right is None:
            return 0.0
        
        left_x = left[-1][0]
        right_x = right[-1][0]
        lane_center = (left_x + right_x) / 2
        car_center = self.w / 2
        
        # Convert to meters
        deviation = (lane_center - car_center) * self.xm_per_pix
        return deviation
    
    def _calculate_confidence(self, left, right, lines) -> float:
        """Calculate detection confidence"""
        conf = 0.0
        if lines is not None:
            conf += min(len(lines) / 20, 0.4)
        if left is not None:
            conf += 0.3
        if right is not None:
            conf += 0.3
        return conf
    
    def visualize(self, frame: np.ndarray, result: DetectionResult) -> np.ndarray:
        """Visualize hierarchical detection results"""
        vis = cv2.resize(frame, (self.w, self.h))
        
        # Show road mask (subtle)
        road_overlay = cv2.cvtColor(result.road.mask, cv2.COLOR_GRAY2BGR)
        road_overlay[:, :, 1] = result.road.mask  # Green channel
        vis = cv2.addWeighted(vis, 1, road_overlay, 0.15, 0)
        
        # Draw lanes
        lanes = result.lanes
        if lanes.left is not None:
            cv2.polylines(vis, [lanes.left], False, (0, 255, 0), 3)
        if lanes.right is not None:
            cv2.polylines(vis, [lanes.right], False, (0, 255, 0), 3)
        if lanes.center is not None:
            cv2.polylines(vis, [lanes.center], False, (0, 255, 255), 2)
        
        # Fill lane area
        if lanes.left is not None and lanes.right is not None:
            pts = np.vstack([lanes.left, lanes.right[::-1]])
            overlay = vis.copy()
            cv2.fillPoly(overlay, [pts], (0, 200, 0))
            cv2.addWeighted(vis, 0.85, overlay, 0.15, 0, vis)
        
        # Info panel with hierarchical data
        self._draw_info_panel(vis, result)
        
        return vis
    
    def _draw_info_panel(self, vis, result):
        """Draw comprehensive info panel"""
        # Semi-transparent background
        cv2.rectangle(vis, (5, 5), (300, 130), (0, 0, 0), -1)
        cv2.rectangle(vis, (5, 5), (300, 130), (0, 255, 0), 2)
        
        info = [
            f"FPS: {result.fps:.1f} | Latency: {result.latency_ms:.1f}ms",
            f"Road: {result.road.type} (Q: {result.road.quality:.0%})",
            f"Confidence: {result.lanes.confidence:.0%}",
            f"Deviation: {result.lanes.deviation:.2f}m",
            f"L-Curve: {result.lanes.left_curve:.0f}m",
            f"R-Curve: {result.lanes.right_curve:.0f}m"
        ]
        
        for i, text in enumerate(info):
            cv2.putText(vis, text, (15, 25 + i*18), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
