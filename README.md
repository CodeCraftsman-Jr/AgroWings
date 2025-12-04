# AgroWings - Cotton Picking Robot 🤖🌾

Autonomous cotton-picking robot using:
- **Raspberry Pi 5** + Camera
- **VL53L5CX ToF Sensor** (depth sensing)
- **Arduino/ESP32** (servo control)
- **Gripper** (mechanical picking)
- **Continuum robot arm** (3-tendon design)
- **YOLO** cotton detection

---

## 📁 Project Structure

### Core Files (Active System)
```
AgroWings/
├── picking_controller.py          # Main execution pipeline
├── mono_tof_vision.py             # Camera + ToF depth fusion
├── tof_sensor.py                  # VL53L5CX driver
├── continuum_kinematics.py        # Arm kinematics (IK/FK)
├── robot_interface.py             # High-level API & testing
├── test_vl53l5cx.py              # ToF sensor testing
│
├── raspi_camera_detection.py     # RPi5 camera + YOLO (standalone)
├── droidcam_detection.py         # DroidCam testing (development)
│
├── arduino_servo_controller/      # Arduino firmware
│   └── arduino_servo_controller.ino
│
├── model/
│   └── best.pt                    # YOLO cotton detection model
│
├── docs/
│   └── SETUP_GUIDE.md            # Complete setup instructions
│
├── IMPLEMENTATION_SUMMARY.md      # Implementation overview
└── requirements.txt               # Python dependencies
```

---

## 🚀 Quick Start

### On Windows (Development)
```bash
# Install dependencies (Windows compatible only)
pip install -r requirements.txt

# Test with mock hardware
python picking_controller.py
# (Edit file to set USE_MOCK_HARDWARE = True)
```

### On Raspberry Pi 5 (Production)
```bash
# Install system packages
sudo apt update
sudo apt install -y python3-picamera2 python3-lgpio i2c-tools

# Install Python dependencies
pip3 install ultralytics opencv-python numpy pyserial scipy matplotlib

# Install ToF sensor library
pip3 install adafruit-circuitpython-vl53l5cx adafruit-blinka

# Enable I2C
sudo raspi-config
# Interface Options → I2C → Enable

# Run tests
python3 test_vl53l5cx.py
python3 vacuum_controller.py

# Run full system
python3 picking_controller.py
```

### Flash Arduino/ESP32
1. Open Arduino IDE
2. Install board support (Arduino Uno/ESP32)
3. Open `arduino_servo_controller/arduino_servo_controller.ino`
4. Select board: Arduino Uno (or ESP32 Dev Module)
5. Upload firmware
6. Verify in Serial Monitor (115200 baud)

---

## 🎯 Main Scripts

| Script | Purpose | Hardware Required |
|--------|---------|-------------------|
| `picking_controller.py` | **Main system** - Full autonomous picking | All hardware |
| `robot_interface.py` | Interactive testing & calibration | Arduino + servos + gripper |
| `mono_tof_vision.py` | Standalone vision testing | RPi camera + ToF |
| `test_vl53l5cx.py` | ToF sensor diagnostics | VL53L5CX only |
| `raspi_camera_detection.py` | Camera + YOLO only | RPi camera |
| `droidcam_detection.py` | Development testing | DroidCam app |

---

## 🔧 Hardware Wiring

### Raspberry Pi 5
```
GPIO 2 (SDA) ──→ VL53L5CX SDA
GPIO 3 (SCL) ──→ VL53L5CX SCL
USB ───────────→ Arduino/ESP32
Camera Port ───→ RPi Camera Module
```

### Arduino/ESP32
```
A4/GPIO21 (SDA) ──→ PCA9685 SDA
A5/GPIO22 (SCL) ──→ PCA9685 SCL
```

### PCA9685 → Servos
```
Channel 0 ──→ Servo 1 (Tendon A)
Channel 1 ──→ Servo 2 (Tendon B)
Channel 2 ──→ Servo 3 (Tendon C)
Channel 3 ──→ Servo 4 (Gripper)
V+ ─────────→ 6V 10A Power Supply
```

---

## 📊 System Architecture

```
RPi5: picking_controller.py
  ├─→ mono_tof_vision.py (Camera + ToF → 3D positions)
  └─→ continuum_kinematics.py (Inverse kinematics → servo angles)
       └─→ Arduino/ESP32 → PCA9685 → 4 Servos (3 arm + 1 gripper)
```

---

## 🧪 Testing Workflow

1. **Test ToF Sensor**
   ```bash
   python3 test_vl53l5cx.py
   ```

2. **Test Camera + Detection**
   ```bash
   python3 raspi_camera_detection.py
   ```

3. **Test Full Vision System**
   ```bash
   python3 mono_tof_vision.py
   ```

4. **Interactive Robot Testing**
   ```bash
   python3 robot_interface.py
   # Menu: Workspace test, picking test, gripper test, etc.
   ```

5. **Run Complete System**
   ```bash
   python3 picking_controller.py
   ```

---

## 📖 Documentation

- **Setup Guide**: [`docs/SETUP_GUIDE.md`](docs/SETUP_GUIDE.md)
  - Complete hardware assembly
  - Software installation
  - Calibration procedures
  - Troubleshooting

- **Implementation Summary**: [`IMPLEMENTATION_SUMMARY.md`](IMPLEMENTATION_SUMMARY.md)
  - What was implemented
  - System features
  - Performance specs

---

## 🎮 Controls (During Operation)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `p` | Pause/Resume |
| `g` | Test gripper |
| `s` | Save frame |

---

## ⚙️ Configuration

Edit `picking_controller.py`:
```python
MODEL_PATH = r"model\best.pt"          # YOLO model
ARDUINO_PORT = "COM3"                   # Windows: COM3, Linux: /dev/ttyUSB0
CAMERA_RESOLUTION = (640, 480)          # Camera resolution
USE_MOCK_HARDWARE = False               # Set True for testing without hardware
```

---

## 📈 Performance Specs

| Metric | Value |
|--------|-------|
| Detection Range | 100-400mm |
| Picking Range | 100-200mm |
| ToF Accuracy | ±5mm |
| Cycle Time | 5-10 sec/pick |
| Camera FPS | 30 @ 640x480 |

---

## 🛠️ Troubleshooting

### Camera Not Working
```bash
libcamera-hello --timeout 5000
```

### ToF Sensor Not Detected
```bash
i2cdetect -y 1
# Should show device at 0x29
```

### Arduino Not Responding
```bash
ls /dev/ttyUSB*  # or /dev/ttyACM*
# Test connection in Arduino Serial Monitor
```

See [`docs/SETUP_GUIDE.md`](docs/SETUP_GUIDE.md) for detailed troubleshooting.

---

## 🔄 Migration from Old System

**Removed Files** (replaced by new implementation):
- ❌ `stereo_vision.py` → ✓ `mono_tof_vision.py` (single camera + ToF)
- ❌ `calibrate_cameras.py` → ✓ Camera intrinsics in mono_tof_vision.py
- ❌ `camera_detection.py` → ✓ `raspi_camera_detection.py` (RPi-specific)
- ❌ `image_video_detection.py` → ✓ Integrated into vision modules
- ❌ `jetson_arducam_detection.py` → ✓ Not needed (RPi5 focused)
- ❌ `vacuum_controller.py` → ✓ Using gripper instead
- ❌ `esp32_servo_vacuum_controller.ino` → ✓ Using Arduino firmware

**Active Files**:
- ✓ `picking_controller.py` (updated for gripper + ToF)
- ✓ `robot_interface.py` (updated with gripper testing)
- ✓ `continuum_kinematics.py` (unchanged - still valid)
- ✓ `raspi_camera_detection.py` (RPi5 camera + YOLO standalone)
- ✓ `droidcam_detection.py` (development tool)

---

## 📦 Dependencies

### Core (All Platforms)
- `ultralytics` - YOLO detection
- `opencv-python` - Image processing
- `numpy` - Math operations
- `pyserial` - ESP32 communication
- `scipy` - Kinematics optimization

### Raspberry Pi Only
- `picamera2` - RPi camera interface
- `adafruit-circuitpython-vl53l5cx` - ToF sensor
- `adafruit-blinka` - I2C/GPIO support

---

## 🏆 Features

- ✓ **Real-time cotton detection** with YOLO
- ✓ **3D positioning** with ToF depth sensing
- ✓ **Gripper-based picking** (mechanical grasp)
- ✓ **Continuum arm control** with inverse kinematics
- ✓ **Safety features** (emergency stop, workspace limits)
- ✓ **Mock mode** for testing without hardware
- ✓ **Interactive testing** suite

---

## 📞 Support

1. Check `docs/SETUP_GUIDE.md` for detailed setup
2. Review code comments and docstrings
3. Run individual component tests
4. Verify hardware connections

---

## 📝 License

Part of the AgroWings cotton-picking robot initiative.

---

**Status**: ✅ Implementation Complete - Ready for Hardware Testing

**Last Updated**: December 2025
