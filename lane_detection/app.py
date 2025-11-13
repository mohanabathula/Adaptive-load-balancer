"""
Simple App Runner for Hierarchical Lane Detection
Usage: python app.py [0|video.mp4]
"""

import cv2
import sys
from detector import HierarchicalLaneDetector


def main():
    # Get source
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    source = int(source) if str(source).isdigit() else source
    
    # Open video
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Error: Cannot open {source}")
        return
    
    print("="*60)
    print("HIERARCHICAL LANE DETECTION")
    print("="*60)
    print(f"Source: {source}")
    print("Controls: Q-Quit | P-Pause | S-Save")
    print("="*60)
    
    # Initialize detector
    detector = HierarchicalLaneDetector()
    paused, count = False, 0
    
    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Hierarchical detection
            result = detector.detect(frame)
            vis = detector.visualize(frame, result)
            count += 1
            
            # Print info
            if count % 30 == 0:
                print(f"Frame {count} | FPS: {result.fps:.1f} | "
                      f"Road: {result.road.type} | "
                      f"Deviation: {result.lanes.deviation:.2f}m")
        
        cv2.imshow('Hierarchical Lane Detection', vis)
        
        key = cv2.waitKey(1 if not paused else 0) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('p'):
            paused = not paused
        elif key == ord('s'):
            cv2.imwrite(f'detection_{count}.jpg', vis)
            print(f"Saved detection_{count}.jpg")
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"\nProcessed {count} frames")


if __name__ == '__main__':
    main()
