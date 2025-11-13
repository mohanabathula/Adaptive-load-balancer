# 🚦 Traffic Signal Detection - Quick Start Guide

## What You Have

A **two-stage traffic signal detection system** that intelligently identifies traffic lights and their active state while filtering out false positives.

## Files in This Project

### Core Files (Required)
- **traffic_signal_detector.py** - Main detection system
- **requirements.txt** - Python packages to install

### Helper Files
- **quick_start.py** - Interactive menu
- **test_system.py** - Test camera and system
- **sample_traffic_light.jpg** - Sample image for testing

### Documentation
- **README.md** - Complete documentation (start here!)
- **IMPROVEMENTS.md** - Technical details about two-stage detection
- **CAMERA_SETUP.md** - macOS camera permission guide

## 🚀 Get Started in 3 Steps

### 1. Install Required Packages
```bash
pip install -r requirements.txt
```

This installs:
- **opencv-python** (version 4.8.0+) - Computer vision library
- **numpy** (version 1.24.0+) - Numerical computing library

### 2. Grant Camera Access (macOS Only)

Go to: **System Settings → Privacy & Security → Camera**
- Enable **Terminal** (or iTerm2)
- Restart Terminal

### 3. Run the Detector
```bash
python traffic_signal_detector.py
```

Press **'q'** to quit

## 📦 What Gets Installed

When you run `pip install -r requirements.txt`:

```
opencv-python>=4.8.0    (~60MB)
  ├── Uses: Image processing, circle detection, color analysis
  └── Why: Core computer vision functionality

numpy>=1.24.0          (~15MB)
  ├── Uses: Array operations, mathematical computations
  └── Why: Fast numerical processing
```

**Total download size**: ~75MB

## 🎯 How It Works

### Two-Stage Detection:

1. **Stage 1**: Find traffic light structure
   - Detects circles using Hough Transform
   - Groups vertically aligned circles
   - Requires 2+ circles in vertical pattern

2. **Stage 2**: Identify active light
   - Checks brightness (must be ON)
   - Analyzes color (red/yellow/green)
   - Calculates confidence score

### Why Two Stages?

❌ **Without**: Detects ANY red/yellow/green object
✅ **With**: Only detects actual traffic light patterns

## 📊 What You'll See

```
┌─────────────────────────────────────┐
│  FPS: 22.3                          │
│  Signal State:                      │
│  RED                                │
│  Press 'q' to quit                  │
│                                     │
│            ⬜                        │
│           ┌──┐                      │
│           │🔴│  ← Active light      │
│           ├──┤                      │
│           │⚫│                      │
│           ├──┤                      │
│           │⚫│                      │
│           └──┘                      │
│         3 lights                    │
│                                     │
│  Detected: 1 traffic light(s)       │
└─────────────────────────────────────┘
```

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| Camera not working | Check CAMERA_SETUP.md |
| No detection | Move closer (5-20m), ensure good lighting |
| False positives | Read IMPROVEMENTS.md for tuning |
| Low FPS | Reduce resolution in code |

## 📖 Learn More

- **Full documentation**: Read [README.md](README.md)
- **Technical details**: Read [IMPROVEMENTS.md](IMPROVEMENTS.md)
- **Camera issues**: Read [CAMERA_SETUP.md](CAMERA_SETUP.md)

## ✅ System Requirements

- Python 3.7+
- Webcam or video file
- ~75MB free disk space (for packages)
- macOS/Linux/Windows

## 🎓 Testing Without Camera

If you don't have camera access:

1. Use the sample image:
   ```python
   # Edit traffic_signal_detector.py
   # Change video_source to an image or video file
   ```

2. Or run the test:
   ```bash
   python test_system.py
   ```

## 💡 Pro Tips

1. **Best Results**: Point camera at traffic light from 5-20 meters
2. **Daytime**: Works best in good lighting conditions
3. **Stable Camera**: Mount or hold camera steady
4. **Front View**: Face the traffic light directly

---

**Ready? Start with:**
```bash
pip install -r requirements.txt
python traffic_signal_detector.py
```

Happy detecting! 🚦
