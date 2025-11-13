"""
Test script for traffic signal detection
This script helps you test the detector with a sample video or webcam
"""

import cv2
import sys
import os

def test_camera():
    """Test if camera is accessible"""
    print("Testing camera access...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Cannot access camera")
        print("Trying alternative camera indices...")
        
        for i in range(1, 5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"✅ Camera found at index {i}")
                cap.release()
                return i
        
        print("❌ No camera found")
        return None
    else:
        print("✅ Camera accessible at index 0")
        cap.release()
        return 0

def test_opencv():
    """Test OpenCV installation"""
    print("\nTesting OpenCV installation...")
    try:
        print(f"✅ OpenCV version: {cv2.__version__}")
        return True
    except Exception as e:
        print(f"❌ OpenCV error: {e}")
        return False

def create_sample_image():
    """Create a sample traffic light image for testing"""
    import numpy as np
    
    # Create a black image
    img = np.zeros((400, 300, 3), dtype=np.uint8)
    
    # Draw traffic light box
    cv2.rectangle(img, (100, 50), (200, 350), (100, 100, 100), -1)
    
    # Draw red light
    cv2.circle(img, (150, 100), 30, (0, 0, 255), -1)
    
    # Draw yellow light
    cv2.circle(img, (150, 200), 30, (0, 255, 255), -1)
    
    # Draw green light (brighter)
    cv2.circle(img, (150, 300), 30, (0, 255, 0), -1)
    
    # Save image
    cv2.imwrite('sample_traffic_light.jpg', img)
    print("✅ Sample traffic light image created: sample_traffic_light.jpg")
    
    return img

def main():
    """Main test function"""
    print("="*60)
    print("Traffic Signal Detection - System Test")
    print("="*60)
    
    # Test OpenCV
    if not test_opencv():
        print("\n❌ Please install OpenCV: pip install opencv-python")
        sys.exit(1)
    
    # Test camera
    camera_index = test_camera()
    
    # Create sample image
    print("\nCreating sample traffic light image...")
    sample_img = create_sample_image()
    
    # Display sample
    print("\nDisplaying sample image (press any key to close)...")
    cv2.imshow('Sample Traffic Light', sample_img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
    
    print("\n" + "="*60)
    print("System Test Complete!")
    print("="*60)
    
    if camera_index is not None:
        print(f"\n✅ You can run the detector with camera index {camera_index}")
        print(f"   python traffic_signal_detector.py")
    else:
        print("\n⚠️  No camera detected. You can still test with video files.")
    
    print("\nNext steps:")
    print("1. Run: python traffic_signal_detector.py")
    print("2. Point your camera at a traffic light")
    print("3. Or test with the sample image or a video file")
    print("4. Press 'q' to quit the detector")

if __name__ == "__main__":
    main()
