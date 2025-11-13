# Camera Setup for macOS

## Camera Permission Issue

You're seeing this error because macOS requires explicit permission for apps to access the camera.

## ✅ How to Grant Camera Access

### Method 1: System Settings (Recommended)

1. Open **System Settings** (or System Preferences)
2. Go to **Privacy & Security**
3. Click on **Camera**
4. Find **Terminal** (or **iTerm2** if you're using that)
5. **Enable the checkbox** next to Terminal
6. Close Terminal completely and reopen it
7. Run the detection script again

### Method 2: Command Line Prompt

1. Run the script again - macOS might show a permission dialog
2. Click **OK** or **Allow** when prompted
3. If no dialog appears, use Method 1 above

## 🔄 After Granting Permission

Once camera access is granted, run:

```bash
python traffic_signal_detector.py
```

Or:

```bash
python quick_start.py
```

## 🧪 Test Camera Access

To verify camera works:

```bash
python test_system.py
```

## 📹 Alternative: Use a Video File

If you don't want to use the camera, you can test with a video file:

1. Download a traffic light video or use any video file
2. Edit `traffic_signal_detector.py` (around line 200):
   ```python
   detector.run(video_source='path/to/your/video.mp4')
   ```
3. Run the script

## 🎥 Using Sample Image

A sample traffic light image was created at:
- `sample_traffic_light.jpg`

You can test detection with static images by modifying the code to load images instead of video.

---

**After setting up permissions, come back and run the detector!** 🚦
