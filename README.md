# Cotton Picking Robot - Full System

## 📋 Overview
Autonomous cotton-picking robot system with:
- **Dual stereo cameras** for 3D vision and depth perception
- **YOLO AI model** to detect ripe cotton
- **Continuum tendon-driven robot arm** for flexible picking
- **Arduino + PCA9685** servo control system
- **Complete picking pipeline** from detection to harvest

## 🎯 Available Scripts

### Detection & Testing

#### 1. **camera_detection.py** - Real-time Camera Detection (Single Camera)
   - Detects cotton in real-time using your camera
   - Shows live detections with bounding boxes
   - Counts ready vs not-ready cotton

   **Usage:**
   ```bash
   python camera_detection.py
   ```

#### 2. **image_video_detection.py** - Process Images/Videos
   - Process single images, videos, or folders
   
   **Usage:**
   ```bash
   python image_video_detection.py
   ```

#### 3. **test_camera.py** - Test Camera Connection
   - Detects available cameras and their indices
   
   **Usage:**
   ```bash
   python test_camera.py
   ```

### Stereo Vision System

#### 4. **stereo_vision.py** - Stereo Camera & Depth Mapping
   - Auto-detects Arducam or generic USB stereo cameras
   - Computes depth maps using StereoSGBM
   - Converts 2D detections to 3D coordinates
   
   **Test stereo cameras:**
   ```bash
   python stereo_vision.py
   ```
   
   **Controls:**
   - 'q' - Quit
   - 'd' - Show depth map
   - 's' - Save frames

#### 5. **calibrate_cameras.py** - Camera Calibration Utility
   - Interactive calibration for stereo camera pairs
   - Generates checkerboard pattern for printing
   - Saves calibration data for accurate depth measurement
   
   **Usage:**
   ```bash
   python calibrate_cameras.py
   ```
   
   **Steps:**
   1. Print checkerboard pattern (generated as PDF)
   2. Mount on flat surface
   3. Capture 20+ images at different angles/distances
   4. Calibration computed and saved automatically

### Robot Control

#### 6. **continuum_kinematics.py** - Robot Arm Kinematics
   - Constant-curvature kinematics solver (Webster et al.)
   - Forward/inverse kinematics for 3-tendon continuum arm
   - Workspace validation
   
   **Test kinematics:**
   ```bash
   python continuum_kinematics.py
   ```

#### 7. **picking_controller.py** - Integrated Picking Pipeline
   - **Main autonomous picking system**
   - Combines stereo vision + YOLO + kinematics + Arduino
   - Detects cotton → computes 3D position → moves arm → picks
   
   **Usage:**
   ```bash
   python picking_controller.py
   ```
   
   **Controls:**
   - 'q' - Quit
   - 'p' - Pause/resume
   - 'd' - Toggle depth visualization
   - 's' - Save frame

#### 8. **robot_interface.py** - High-Level Robot Control
   - Manual control mode
   - Workspace testing
   - Calibration utilities
   - Safety features and emergency stop
   
   **Usage:**
   ```bash
   python robot_interface.py
   ```
   
   **Features:**
   - Test workspace coverage
   - Test picking sequences
   - Manual keyboard control
   - Home position calibration

### Arduino Firmware

#### 9. **arduino_servo_controller.ino** - Arduino Controller
   - Controls 4 servos via PCA9685 PWM driver
   - Receives serial commands from laptop
   - Format: `S1:90,S2:45,S3:120,S4:0`
   
   **Upload to Arduino:**
   1. Open in Arduino IDE
   2. Install library: `Adafruit_PWMServoDriver`
   3. Select board (Arduino Uno/Nano)
   4. Select COM port
   5. Upload

## 🔧 Setup

### 1. Install Python Requirements
```bash
pip install -r requirements.txt
```

**Packages installed:**
- `ultralytics` - YOLO model
- `opencv-python` - Computer vision
- `numpy` - Numerical computing
- `scipy` - Optimization (for inverse kinematics)
- `pyserial` - Arduino communication
- `keyboard` - Manual control interface
- `matplotlib` - Visualization

### 2. Arduino Setup

**Hardware connections:**
```
PCA9685 → Arduino:
  SDA → A4
  SCL → A5
  VCC → 5V
  GND → GND
  
Power Supply (6V 10A) → PCA9685 V+
Common ground between Arduino and power supply

Servos → PCA9685 channels 0-3
```

**Upload firmware:**
1. Install Arduino IDE
2. Install library: `Adafruit PWM Servo Driver Library`
3. Open `arduino_servo_controller/arduino_servo_controller.ino`
4. Upload to Arduino
5. Note the COM port (e.g., COM3)

### 3. Camera Calibration

**For accurate depth measurement, calibrate stereo cameras:**

```bash
python calibrate_cameras.py
```

1. Select "Generate checkerboard pattern" → prints PDF
2. Print checkerboard at actual size (no scaling!)
3. Mount on flat, rigid surface
4. Select "Capture new calibration images"
5. Capture 20+ images at various angles and distances
6. Calibration automatically saved to `calibration_data/`

### 4. Test Individual Systems

**Test cameras:**
```bash
python test_camera.py
```

**Test stereo vision:**
```bash
python stereo_vision.py
```

**Test kinematics:**
```bash
python continuum_kinematics.py
```

**Test Arduino connection:**
- Open Arduino IDE Serial Monitor (115200 baud)
- Type: `S1:90,S2:90,S3:90,S4:90`
- Should see "OK" response

### 5. Run Complete System

**Update configuration in `picking_controller.py`:**
```python
MODEL_PATH = r"model\best.pt"
CALIBRATION_FILE = r"calibration_data\stereo_calibration.json"
ARDUINO_PORT = "COM3"  # Your Arduino port
```

**Run:**
```bash
python picking_controller.py
```

## 📁 Folder Structure
```
AgroWings/
├── model/
│   └── best.pt                         # YOLO cotton detection model
├── calibration_data/                   # Stereo calibration files
│   ├── checkerboard_pattern.pdf       # Print this for calibration
│   └── stereo_calibration.json        # Saved calibration
├── arduino_servo_controller/
│   └── arduino_servo_controller.ino   # Arduino firmware
├── output/                             # Processed images/videos
├── Combined_Cotton_Dataset/            # Training dataset
│
├── camera_detection.py                 # Single camera detection
├── image_video_detection.py            # Image/video processing
├── test_camera.py                      # Camera testing
├── stereo_vision.py                    # Stereo vision system
├── calibrate_cameras.py                # Calibration utility
├── continuum_kinematics.py             # Robot kinematics
├── picking_controller.py               # Main picking pipeline
├── robot_interface.py                  # High-level control
└── requirements.txt                    # Python dependencies
```


## 🎥 Hardware Requirements

### Required Components

**Vision System:**
- 2× Arducam Stereo USB Camera (OV9281, OV2311, or similar) - ₹9,000
  - Alternative: 2× Generic USB webcams in stereo mount
- USB 3.0 cables - ₹200

**Control Electronics:**
- 1× Arduino Uno or Nano - ₹300
- 1× PCA9685 16-channel PWM servo driver - ₹300
- 1× 6V 10A DC power supply - ₹800
- Jumper wires, headers - ₹200

**Robot Arm:**
- 3× MG996R or DS3225 high-torque servos (for tendons) - ₹1,200-₹1,800
- 1× SG90 or MG90S servo (for gripper) - ₹150-₹180
- Fishing line / nylon tendons (strong) - ₹80
- 3D printed parts (see STL files in plan.md) - ₹500

**Total:** ≈ ₹14,500-₹16,000

### Wiring Diagram

```
         [Laptop]
             │ USB 3.0
  ┌──────────┴───────────┐
  │                      │
[Camera L]          [Camera R]
  USB 3.0             USB 3.0
  
  [Laptop] ──USB Serial──> [Arduino Uno]
                               │ I2C (SDA/SCL)
                               ▼
                         [PCA9685 Board]
                    ┌───────┼────────┬─────┐
                    ▼       ▼        ▼     ▼
                Servo1  Servo2  Servo3  Servo4
               (Arm)    (Arm)   (Arm)  (Gripper)
                    └───────┴────────┴─────┘
                   6V 10A Power Supply
                   
* Common ground between Arduino and power supply
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


## 🚀 Quick Start Guide

### Basic Testing (No Hardware)
```bash
# 1. Test YOLO detection on images
python image_video_detection.py

# 2. Test kinematics calculations
python continuum_kinematics.py
```

### With Cameras Only
```bash
# 1. Find available cameras
python test_camera.py

# 2. Test single camera detection
python camera_detection.py

# 3. Test stereo vision (requires 2 cameras)
python stereo_vision.py

# 4. Calibrate stereo cameras
python calibrate_cameras.py
```

### Full System (All Hardware)
```bash
# 1. Upload Arduino firmware
# Open arduino_servo_controller.ino in Arduino IDE and upload

# 2. Test servos via Serial Monitor
# Type: S1:90,S2:90,S3:90,S4:90

# 3. Update configuration
# Edit picking_controller.py with your COM port

# 4. Run complete system
python picking_controller.py

# 5. Or use high-level interface
python robot_interface.py
```

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


## 📞 Common Issues

**Issue: "Camera index out of range"**
- Solution: Run `python test_camera.py` to find correct indices

**Issue: "Model not found"**
- Solution: Ensure `best.pt` is in the `model/` folder

**Issue: "No detections"**
- Solution: Lower `CONFIDENCE_THRESHOLD` or improve lighting

**Issue: "Slow detection"**
- Solution: Reduce camera resolution in script or use GPU

**Issue: "Arduino not responding"**
- Solution: Check COM port, ensure correct baud rate (115200)
- Install `Adafruit_PWMServoDriver` library in Arduino IDE

**Issue: "Depth map looks wrong"**
- Solution: Run camera calibration with `calibrate_cameras.py`
- Ensure cameras are properly aligned and fixed in position

**Issue: "Inverse kinematics fails"**
- Solution: Target may be out of workspace
- Check workspace bounds in `robot_interface.py`
- Try positions closer to home (0, 0, 150)

**Issue: "Servos not moving"**
- Solution: Check power supply (6V 10A)
- Verify PCA9685 connections (I2C wiring)
- Test with Arduino Serial Monitor commands

**Issue: "Import error: No module named..."**
- Solution: Install all requirements: `pip install -r requirements.txt`

## 🔬 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    PICKING CONTROLLER                       │
│  (picking_controller.py - Main orchestration)               │
└──────┬────────────────┬──────────────┬────────────┬─────────┘
       │                │              │            │
   ┌───▼────┐     ┌─────▼─────┐  ┌────▼────┐  ┌───▼────┐
   │ YOLO   │     │  Stereo   │  │Continuum│  │Arduino │
   │ Model  │     │  Vision   │  │Kinematic│  │ Serial │
   │        │     │           │  │         │  │        │
   │best.pt │     │stereo_    │  │continuum│  │ COM3   │
   │        │     │vision.py  │  │_kine... │  │115200  │
   └────────┘     └───────────┘  └─────────┘  └────────┘
       │                │              │            │
   Detection      3D Position    Servo Angles   Commands
   (2D bbox)      (X,Y,Z mm)     (0-180°)       (S1:90...)
       │                │              │            │
       └────────────────┴──────────────┴────────────┘
                           │
                    Target Cotton
                   Picked Successfully!
```

## 📚 Research References

**Continuum Robot Kinematics:**
- Webster & Jones (2010) - "Design and Kinematic Modeling of Constant Curvature Continuum Robots"
- Implementation in `continuum_kinematics.py`

**Stereo Vision:**
- OpenCV StereoSGBM algorithm
- Camera calibration using Zhang's method
- Implementation in `stereo_vision.py`

**Object Detection:**
- YOLOv8/v11 - Ultralytics implementation
- Trained on custom cotton dataset

## 🛠️ Development & Testing

### Test Individual Components

```bash
# Test stereo vision
python -c "from stereo_vision import StereoVisionSystem; s=StereoVisionSystem(); print(s.detect_cameras())"

# Test kinematics forward
python -c "from continuum_kinematics import ContinuumKinematics; import numpy as np; k=ContinuumKinematics(); print(k.forward_kinematics(np.array([10,10,10])))"

# Test Arduino connection
python -c "import serial; s=serial.Serial('COM3', 115200); print(s.readline())"
```

### Workspace Limits

Default workspace (mm):
- X: [-60, 60]
- Y: [-60, 60]  
- Z: [50, 150]

Adjust in `robot_interface.py` based on your arm configuration.

### Servo Calibration

Use `robot_interface.py` → Option 4 to find neutral positions.
Update servo offsets in `continuum_kinematics.py`:
```python
servo_offset = [90, 90, 90]  # Your calibrated values
```

## 🎓 Next Steps

1. **Hardware Assembly**
   - Follow assembly instructions in `plan.md`
   - Print STL files (continuum arm, camera mounts, gripper)
   - Wire electronics per diagram

2. **Software Testing**
   - Test each component individually
   - Calibrate stereo cameras
   - Calibrate servo home positions

3. **Integration**
   - Run picking_controller.py
   - Start with manual mode for safety
   - Gradually enable autonomous picking

4. **Optimization**
   - Tune YOLO confidence threshold
   - Adjust stereo matcher parameters
   - Optimize picking trajectory

## 📄 License & Credits

**Cotton Detection Model:** Custom trained YOLOv8
**Continuum Kinematics:** Based on Webster et al. research
**Arduino Controller:** Custom implementation
**Stereo Vision:** OpenCV-based

Built for AgroWings project - Autonomous Agricultural Robotics

---

**For questions or issues, check the plan.md file for detailed specifications and assembly instructions.**
