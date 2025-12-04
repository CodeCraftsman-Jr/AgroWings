# Cotton Picking Robot - Implementation Guide

## Overview

This guide covers the complete implementation of your cotton-picking robot system using:
- **Raspberry Pi 5** with camera
- **VL53L5CX ToF sensor** for depth sensing
- **ESP32-S3** for servo and vacuum control
- **Vacuum suction system** for cotton picking
- **Continuum robot arm** with 3 servos

---

## Hardware Components

### Core Components
1. **Raspberry Pi 5** (4GB or 8GB)
2. **Raspberry Pi Camera Module** (compatible with RPi5)
3. **VL53L5CX ToF Sensor** (I2C, 8x8 multi-zone ranging)
4. **ESP32-S3 DevKit**
5. **PCA9685 16-Channel PWM Driver**
6. **3x MG996R or DS3225 Servos** (for continuum arm tendons)
7. **12V Mini Vacuum Pump** (diaphragm type, 5-8 L/min)
8. **5V Relay Module** (for pump control)
9. **Solenoid Valve** (optional, 2-way normally closed)
10. **Vacuum Pressure Sensor** (analog output)
11. **Vacuum Cup/Nozzle** (soft silicone, 15-20mm diameter)
12. **Power Supply** (6V 10A for servos, 12V for vacuum)

### Wiring Diagram

#### Raspberry Pi 5 Connections
```
RPi5:
  GPIO 2 (SDA) ───→ VL53L5CX SDA
  GPIO 3 (SCL) ───→ VL53L5CX SCL
  3.3V ───────────→ VL53L5CX VCC
  GND ────────────→ VL53L5CX GND
  USB ────────────→ ESP32-S3 (serial)
  Camera Port ───→ RPi Camera Module
```

#### ESP32-S3 Connections
```
ESP32-S3:
  GPIO 21 (SDA) ──→ PCA9685 SDA
  GPIO 22 (SCL) ──→ PCA9685 SCL
  GPIO 25 ────────→ Relay Module IN (vacuum pump)
  GPIO 26 ────────→ Solenoid Valve Control
  GPIO 35 (ADC) ──← Vacuum Pressure Sensor OUT
  5V ─────────────→ Relay Module VCC
  3.3V ───────────→ PCA9685 VCC
  GND ────────────→ Common Ground
```

#### PCA9685 to Servos
```
PCA9685:
  Channel 0 ──→ Servo 1 (Tendon A)
  Channel 1 ──→ Servo 2 (Tendon B)
  Channel 2 ──→ Servo 3 (Tendon C)
  V+ ─────────→ 6V 10A Power Supply
  GND ────────→ Common Ground
```

#### Vacuum System
```
Vacuum Pump:
  + ──→ Relay NO (normally open)
  - ──→ GND
  
Relay:
  COM ──→ 12V Power Supply
  NO ───→ Pump +
  IN ───→ ESP32 GPIO25
  
Solenoid Valve:
  + ──→ ESP32 GPIO26 (via transistor/MOSFET)
  - ──→ GND
```

---

## Software Setup

### 1. Raspberry Pi 5 Setup

#### Install Operating System
```bash
# Use Raspberry Pi Imager to install Raspberry Pi OS (64-bit)
# Enable SSH and configure WiFi during imaging
```

#### Update System
```bash
sudo apt update
sudo apt upgrade -y
```

#### Install Python Dependencies
```bash
# Install pip if needed
sudo apt install python3-pip -y

# Install system packages
sudo apt install -y \
    python3-opencv \
    python3-numpy \
    python3-picamera2 \
    i2c-tools \
    libcamera-apps

# Navigate to project directory
cd ~/AgroWings

# Install Python requirements
pip3 install -r requirements.txt
```

#### Enable I2C
```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
sudo reboot

# Verify I2C is working
i2cdetect -y 1
# Should show VL53L5CX at address 0x29
```

#### Test Camera
```bash
libcamera-hello --timeout 5000
# Should show camera preview for 5 seconds
```

### 2. ESP32-S3 Setup

#### Install Arduino IDE
1. Download Arduino IDE from https://www.arduino.cc/
2. Install ESP32 board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

#### Install Required Libraries
In Arduino IDE Library Manager (Sketch → Include Library → Manage Libraries):
- **Adafruit PWM Servo Driver Library** (for PCA9685)
- **Wire** (built-in, for I2C)

#### Upload Firmware
1. Open `esp32_servo_vacuum_controller/esp32_servo_vacuum_controller.ino`
2. Select board: Tools → Board → ESP32 Arduino → ESP32S3 Dev Module
3. Select port: Tools → Port → (your ESP32 COM port)
4. Click Upload
5. Open Serial Monitor (115200 baud) to verify:
   ```
   ESP32-S3 Servo & Vacuum Controller Ready
   ```

---

## Testing & Calibration

### Phase 1: Component Testing

#### Test 1: VL53L5CX ToF Sensor
```bash
cd ~/AgroWings
python3 test_vl53l5cx.py
```
Expected output:
- Depth map visualization (8x8 grid)
- Distance readings in millimeters
- Statistics (min, max, mean depth)

#### Test 2: Vacuum System
```bash
python3 vacuum_controller.py
```
Tests:
- ESP32 communication
- Vacuum pump activation
- Pressure sensor readings
- Pick/release sequence

#### Test 3: Camera + YOLO Detection
```bash
python3 raspi_camera_detection.py
```
Should display:
- Live camera feed
- Cotton detections with bounding boxes
- Confidence scores

#### Test 4: Mono+ToF Vision Fusion
```bash
python3 mono_tof_vision.py
```
Should show:
- 2D detections with YOLO
- Depth measurements from ToF
- 3D positions (X, Y, Z) in millimeters

### Phase 2: Robot Interface Testing

```bash
python3 robot_interface.py
```

Menu options:
1. **Test workspace coverage** - Verifies arm can reach random positions
2. **Test picking sequence** - Tests complete pick-and-place cycle
3. **Manual control mode** - Keyboard control for debugging
4. **Calibrate home position** - Find neutral servo positions
5. **Test vacuum system** - Interactive vacuum testing
6. **Move to home** - Return arm to safe position
7. **Emergency stop test** - Verify safety shutdown

### Phase 3: Calibration

#### Camera Calibration (Optional but Recommended)
The system uses default camera intrinsic parameters. For better accuracy:

1. Print a checkerboard pattern (9x6 squares, 25mm each)
2. Capture 20+ images from different angles
3. Run calibration (requires separate calibration script)
4. Update camera parameters in `mono_tof_vision.py`:

```python
vision.set_camera_parameters(
    focal_length_x=YOUR_FX,
    focal_length_y=YOUR_FY,
    principal_point_x=YOUR_CX,
    principal_point_y=YOUR_CY
)
```

#### Servo Home Position Calibration
Run robot interface calibration:
```bash
python3 robot_interface.py
# Select option 4: Calibrate home position
# Adjust servos until arm is perfectly straight
# Note the servo angles displayed
```

Update `continuum_kinematics.py` with calibrated offsets if needed.

---

## Running the System

### Full Autonomous Operation

```bash
cd ~/AgroWings
python3 picking_controller.py
```

**What happens:**
1. Initializes camera, ToF sensor, vacuum, and kinematics
2. Starts real-time cotton detection
3. For each detected ready cotton:
   - Gets 3D position (camera + ToF depth)
   - Plans arm trajectory (inverse kinematics)
   - Moves arm to position
   - Activates vacuum and verifies suction
   - Retracts to home position
   - Releases cotton into container
4. Continues until stopped (press 'q')

**Keyboard controls during operation:**
- `q` - Quit
- `p` - Pause/Resume
- `v` - Test vacuum system
- `s` - Save current frame

### Mock Mode (Testing Without Hardware)

For development/testing without physical hardware:

```bash
# Edit picking_controller.py
# Set: USE_MOCK_HARDWARE = True

python3 picking_controller.py
```

This uses:
- Mock ToF sensor (simulated depth)
- Mock vacuum controller (simulated suction)
- Real camera and YOLO detection

---

## Troubleshooting

### Issue: Camera not detected
```bash
# Check camera connection
libcamera-hello

# If not working, enable legacy camera
sudo raspi-config
# Interface Options → Legacy Camera → Enable
```

### Issue: VL53L5CX not detected
```bash
# Check I2C connection
i2cdetect -y 1

# Should show device at 0x29
# If not, check wiring (SDA, SCL, VCC, GND)
```

### Issue: ESP32 not responding
```bash
# Check serial port
ls /dev/ttyUSB* /dev/ttyACM*

# Test with screen
sudo screen /dev/ttyUSB0 115200

# Send: PING
# Should receive: OK
```

### Issue: Weak vacuum / No suction
1. Check for air leaks in hose connections
2. Verify vacuum cup is properly seated
3. Test pump directly (bypass relay)
4. Check pressure sensor calibration
5. Ensure valve is opening (if using solenoid)

### Issue: Servo jitter or erratic movement
1. Check power supply capacity (6V 10A minimum for 3 servos)
2. Add capacitors (1000µF) across servo power
3. Verify common ground between all components
4. Reduce servo speed in kinematics

### Issue: Inaccurate depth measurements
1. Calibrate ToF sensor (check for consistent readings)
2. Calibrate camera intrinsics
3. Verify proper ToF zone-to-pixel mapping
4. Test at various distances (100-400mm range)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Raspberry Pi 5                        │
│  ┌───────────────────────────────────────────────────┐  │
│  │         picking_controller.py (Main)              │  │
│  │  Orchestrates all subsystems                      │  │
│  └───────────────────────────────────────────────────┘  │
│           │              │              │                │
│           ▼              ▼              ▼                │
│  ┌──────────────┐ ┌─────────────┐ ┌──────────────┐    │
│  │mono_tof      │ │vacuum_      │ │continuum_    │    │
│  │_vision.py    │ │controller.py│ │kinematics.py │    │
│  │              │ │             │ │              │    │
│  │• Camera      │ │• Serial cmd │ │• Inverse IK  │    │
│  │• YOLO detect │ │• Pressure   │ │• Forward IK  │    │
│  │• ToF depth   │ │• Safety     │ │• Workspace   │    │
│  └──────────────┘ └─────────────┘ └──────────────┘    │
│        │                │                 │              │
└────────┼────────────────┼─────────────────┼──────────────┘
         │                │                 │
    ┌────▼────┐      ┌───▼───────┐    ┌───▼────┐
    │VL53L5CX │      │  ESP32-S3  │    │ Servo  │
    │ToF      │      │            │    │Commands│
    │Sensor   │      │GPIO25→Pump │    │        │
    │(I2C)    │      │GPIO26→Valve│    │        │
    └─────────┘      │GPIO35←Pres │    │        │
                     │I2C→PCA9685 │◄───┘        │
                     └────────────┘              │
                          │        │             │
                    ┌─────▼───┐ ┌─▼──────────┐  │
                    │Vacuum   │ │PCA9685 PWM │  │
                    │System   │ │Controller  │  │
                    └─────────┘ └────────────┘  │
                                     │           │
                              ┌──────▼───────────▼──┐
                              │ 3 Servos (Arm)      │
                              │ Continuum Tendons   │
                              └─────────────────────┘
```

---

## File Structure

```
AgroWings/
├── picking_controller.py          # Main execution pipeline
├── mono_tof_vision.py             # Camera + ToF fusion
├── vacuum_controller.py           # Vacuum system control
├── tof_sensor.py                  # VL53L5CX driver
├── continuum_kinematics.py        # Arm kinematics
├── robot_interface.py             # High-level API & testing
├── test_vl53l5cx.py              # ToF sensor testing
├── requirements.txt               # Python dependencies
├── model/
│   └── best.pt                    # YOLO cotton detection model
├── esp32_servo_vacuum_controller/
│   └── esp32_servo_vacuum_controller.ino  # ESP32 firmware
└── docs/
    └── SETUP_GUIDE.md             # This file
```

---

## Next Steps

1. **Hardware Assembly**
   - Mount all components securely
   - Wire according to diagrams
   - Test each subsystem independently

2. **Software Installation**
   - Flash ESP32 firmware
   - Install RPi dependencies
   - Run component tests

3. **Calibration**
   - Calibrate home position
   - Test workspace coverage
   - Verify vacuum performance

4. **Field Testing**
   - Test with real cotton plants
   - Tune detection confidence thresholds
   - Optimize picking cycle time
   - Measure success rate

5. **Optimization**
   - Add error recovery logic
   - Implement pick success verification
   - Log performance metrics
   - Battery power optimization

---

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| Detection Range | 100-400mm |
| Picking Range | 100-200mm (configurable) |
| Detection Confidence | 0.5 (50%) minimum |
| ToF Accuracy | ±5mm (typical) |
| Workspace | X: ±60mm, Y: ±60mm, Z: 50-150mm |
| Cycle Time | ~5-10 seconds per pick |
| Vacuum Suction | -40 kPa (good suction threshold) |
| Camera Resolution | 640x480 @ 30fps |
| ToF Resolution | 8x8 zones @ 15Hz |

---

## Safety Features

- **Emergency stop** - Keyboard interrupt or dedicated function
- **Workspace bounds** - Software limits prevent overextension
- **Vacuum timeout** - Auto-shutoff if no commands received
- **Pressure monitoring** - Verifies successful grasp
- **Collision avoidance** - Kinematics checks reachability

---

## Support & Resources

- **YOLO Documentation**: https://docs.ultralytics.com/
- **VL53L5CX Datasheet**: https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html
- **ESP32-S3 Docs**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/
- **Raspberry Pi Camera**: https://www.raspberrypi.com/documentation/accessories/camera.html
- **Continuum Robotics**: Webster & Jones (2010) - Constant Curvature Kinematics

---

## License

This project is part of the AgroWings cotton-picking robot initiative.

---

**Last Updated**: December 2025
**Version**: 1.0
