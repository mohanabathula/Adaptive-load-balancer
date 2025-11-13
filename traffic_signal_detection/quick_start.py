#!/usr/bin/env python3
"""
Quick Start - Traffic Signal Detection
Run this file to quickly test the traffic signal detection system
"""

import sys
import subprocess

def print_header():
    """Print welcome header"""
    print("\n" + "="*70)
    print("🚦  TRAFFIC SIGNAL DETECTION - QUICK START  🚦")
    print("="*70 + "\n")

def check_dependencies():
    """Check if required packages are installed"""
    print("Checking dependencies...\n")
    
    try:
        import cv2
        print(f"✅ OpenCV installed (version {cv2.__version__})")
    except ImportError:
        print("❌ OpenCV not found")
        print("Installing opencv-python...")
        subprocess.run([sys.executable, "-m", "pip", "install", "opencv-python"])
    
    try:
        import numpy
        print(f"✅ NumPy installed (version {numpy.__version__})")
    except ImportError:
        print("❌ NumPy not found")
        print("Installing numpy...")
        subprocess.run([sys.executable, "-m", "pip", "install", "numpy"])
    
    print()

def show_menu():
    """Display menu options"""
    print("\nSelect an option:")
    print("-" * 70)
    print("1. Run Traffic Signal Detector (Webcam)")
    print("2. Run System Test")
    print("3. View Examples")
    print("4. Exit")
    print("-" * 70)

def main():
    """Main function"""
    print_header()
    check_dependencies()
    
    while True:
        show_menu()
        choice = input("\nEnter your choice (1-4): ").strip()
        
        if choice == '1':
            print("\n🚀 Starting Traffic Signal Detector...")
            print("Press 'q' in the video window to quit\n")
            try:
                from traffic_signal_detector import TrafficSignalDetector
                detector = TrafficSignalDetector()
                detector.run(video_source=0)
            except Exception as e:
                print(f"\n❌ Error: {e}")
                print("Make sure your camera is connected and not in use by another application.")
        
        elif choice == '2':
            print("\n🔧 Running System Test...\n")
            try:
                import test_system
                test_system.main()
            except Exception as e:
                print(f"\n❌ Error: {e}")
        
        elif choice == '3':
            print("\n📚 Example Usage:\n")
            print("1. Basic webcam detection:")
            print("   python traffic_signal_detector.py")
            print()
            print("2. Use with video file:")
            print("   Modify traffic_signal_detector.py, line ~200:")
            print("   detector.run(video_source='your_video.mp4')")
            print()
            print("3. Advanced YOLO detection:")
            print("   python yolo_traffic_detector.py")
            print()
            print("4. Custom examples:")
            print("   python examples.py")
            print()
            input("Press Enter to continue...")
        
        elif choice == '4':
            print("\n👋 Goodbye!\n")
            sys.exit(0)
        
        else:
            print("\n❌ Invalid choice. Please enter 1-4.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user. Goodbye!\n")
        sys.exit(0)
