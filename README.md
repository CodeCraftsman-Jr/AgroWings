# Cotton Detection System - Quick Start Guide

## 📋 Overview
This system uses AI to detect cotton and determine if it's ready to pluck or not ready.

## 🎯 Available Scripts

### 1. **camera_detection.py** - Real-time Camera Detection
   - Detects cotton in real-time using your camera
   - Shows live detections with bounding boxes
   - Counts ready vs not-ready cotton

   **Usage:**
   ```bash
   python camera_detection.py
   ```

   **Controls:**
   - Press 'q' to quit
   - Press 's' to save current frame
   - Press 'SPACE' to pause/resume

### 2. **image_video_detection.py** - Process Images/Videos
   - Process single images
   - Process video files
   - Process entire folders of images

   **Usage:**
   ```bash
   python image_video_detection.py
   ```

### 3. **test_camera.py** - Test Camera Connection
   - Tests which cameras are available
   - Helps troubleshoot camera issues

   **Usage:**
   ```bash
   python test_camera.py
   ```

## 🔧 Setup

1. **Install Requirements:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Test Your Camera:**
   ```bash
   python test_camera.py
   ```

3. **Start Detection:**
   ```bash
   python camera_detection.py
   ```

## 📁 Folder Structure
```
Cotton Detection/
├── model/
│   └── best.pt                    # Trained AI model
├── detections/                     # Saved camera captures
├── output/                         # Processed images/videos
├── camera_detection.py             # Real-time camera script
├── image_video_detection.py        # Image/video processing
├── test_camera.py                  # Camera testing tool
└── requirements.txt                # Required packages
```

## 🎥 Camera Troubleshooting

If camera doesn't work:

1. **Run camera test:**
   ```bash
   python test_camera.py
   ```

2. **Check permissions:**
   - Windows: Settings → Privacy → Camera → Allow apps to access camera

3. **Update CAMERA_INDEX:**
   - Open `camera_detection.py`
   - Change `CAMERA_INDEX = 0` to the working camera index

4. **For USB cameras:**
   - Try different USB ports
   - Ensure camera is not used by other apps
   - Check Device Manager for driver issues

## 📊 Detection Classes

- **cotton_ready** (Green) - Cotton ready to pluck
- **cotton_not_ready** (Red) - Cotton not yet ready

## 💡 Tips

1. **Better Detection:**
   - Ensure good lighting
   - Keep camera steady
   - Maintain consistent distance from cotton

2. **Adjust Confidence:**
   - Edit `CONFIDENCE_THRESHOLD` in scripts (default: 0.5)
   - Higher = more strict, Lower = more detections

3. **Save Important Detections:**
   - Press 's' during camera detection to save frames

## 🚀 Quick Test

If you don't have a camera, test with images:

```bash
python image_video_detection.py
# Choose option 3 (folder)
# Use: Combined_Cotton_Dataset/test/images
```

## ⚙️ Configuration

Edit these values at the top of each script:

- `MODEL_PATH` - Path to your trained model
- `CONFIDENCE_THRESHOLD` - Detection confidence (0.0 to 1.0)
- `CAMERA_INDEX` - Camera to use (0, 1, 2, etc.)

## 📞 Common Issues

**Issue: "Camera index out of range"**
- Solution: Run `test_camera.py` to find correct index

**Issue: "Model not found"**
- Solution: Ensure `best.pt` is in the `model/` folder

**Issue: "No detections"**
- Solution: Lower `CONFIDENCE_THRESHOLD` or improve lighting

**Issue: "Slow detection"**
- Solution: Reduce camera resolution in script
- Or use a GPU if available
