# Raspberry Pi Setup Guide for Cotton Detection

## Hardware Requirements

### Recommended Setup
- **Raspberry Pi 4 (4GB or 8GB)** or **Raspberry Pi 5**
- **Pi Camera Module 3** (12MP, best performance)
  - Alternative: Pi Camera Module 2 (8MP)
  - Alternative: Pi HQ Camera (12.3MP)
- **microSD Card** (32GB+ Class 10)
- **Power Supply** (5V/3A minimum)
- **Cooling** (heatsink + fan recommended for AI workloads)

### Optional
- Coral USB Accelerator (for faster inference)
- Touchscreen display (for portable operation)
- Battery pack (for field operation)

## Software Installation

### 1. Install Raspberry Pi OS (64-bit)

```bash
# Use Raspberry Pi Imager
# Download from: https://www.raspberrypi.com/software/
# Select: Raspberry Pi OS (64-bit) - with desktop

# After flashing, boot Pi and complete initial setup
```

### 2. Update System

```bash
# Update package list
sudo apt update
sudo apt upgrade -y

# Install essential tools
sudo apt install -y python3-pip python3-opencv
sudo apt install -y python3-picamera2
```

### 3. Enable Camera

```bash
# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable

# Reboot
sudo reboot

# Test camera after reboot
libcamera-hello
# You should see camera preview for 5 seconds
```

### 4. Install Python Dependencies

```bash
# Create virtual environment (recommended)
python3 -m venv ~/cotton-env
source ~/cotton-env/bin/activate

# Install PyTorch (CPU version for Raspberry Pi)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install Ultralytics YOLO
pip3 install ultralytics

# Install additional dependencies
pip3 install opencv-python picamera2 pillow

# Or use requirements.txt
pip3 install -r requirements.txt
```

### 5. Transfer Project Files

```bash
# Method 1: Using git
cd ~
git clone https://github.com/yourusername/AgroWings.git
cd AgroWings

# Method 2: Using SCP from your PC
# On your PC:
scp -r AgroWings/ pi@raspberrypi.local:~/

# Method 3: Using USB drive
# Copy files to USB drive, then on Pi:
cp -r /media/pi/USB_DRIVE/AgroWings ~/
```

### 6. Test Camera Setup

```bash
cd ~/AgroWings

# Test camera detection
python3 raspi_camera_detection.py
```

## Performance Optimization

### Enable Hardware Acceleration

```bash
# Increase GPU memory
sudo nano /boot/config.txt

# Add or modify:
gpu_mem=256

# Save and reboot
sudo reboot
```

### Optimize Model for Raspberry Pi

```python
# Use smaller YOLO model for faster inference
# In raspi_camera_detection.py, you can export to ONNX or TFLite:

from ultralytics import YOLO

# Load your model
model = YOLO('model/best.pt')

# Export to optimized format
model.export(format='onnx')  # or 'tflite'

# Then load optimized model
model = YOLO('model/best.onnx')
```

### Reduce Resolution for Better FPS

```python
# In raspi_camera_detection.py
CAMERA_WIDTH = 640   # Reduced from 1280
CAMERA_HEIGHT = 480  # Reduced from 720
```

## Camera Module Comparison

| Model | Resolution | FPS | Price | Best For |
|-------|-----------|-----|-------|----------|
| Pi Camera v2 | 8MP | 30fps | $25 | Budget option |
| Pi Camera v3 | 12MP | 50fps | $35 | Best value |
| Pi HQ Camera | 12.3MP | 30fps | $50 | Quality, C/CS mount lenses |

## Physical Mounting

### Use Same 3D Printed Mounts
The mounts in `Materials/mounts/` work for Raspberry Pi too!

**Adjust OpenSCAD parameters:**
```openscad
// For Raspberry Pi Camera v2/v3
camera_pcb_width = 25;
camera_pcb_height = 24;
camera_hole_spacing = 12.5;  // Different from Arducam

// For Pi HQ Camera
camera_pcb_width = 38;
camera_pcb_height = 38;
camera_hole_spacing = 30;
```

### Raspberry Pi to Robot Arm Mount
- Use M2.5 standoffs to attach Pi to robot
- Mount camera separately or use combined mount
- Keep camera CSI cable < 300mm for stability

## Running the Detection System

### Manual Start
```bash
cd ~/AgroWings
source ~/cotton-env/bin/activate  # if using venv
python3 raspi_camera_detection.py
```

### Auto-Start on Boot

```bash
# Create systemd service
sudo nano /etc/systemd/system/cotton-detection.service
```

Add this content:
```ini
[Unit]
Description=Cotton Detection Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/AgroWings
ExecStart=/home/pi/cotton-env/bin/python3 /home/pi/AgroWings/raspi_camera_detection.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable service:
```bash
sudo systemctl enable cotton-detection.service
sudo systemctl start cotton-detection.service

# Check status
sudo systemctl status cotton-detection.service
```

## Troubleshooting

### Camera Not Detected
```bash
# Check camera connection
libcamera-hello

# If fails, check cable connection
# Ensure blue side faces towards Ethernet port

# Check system logs
dmesg | grep camera

# Reinstall camera software
sudo apt install --reinstall rpicam-apps
```

### Low FPS / Slow Detection
```bash
# Check CPU usage
htop

# Monitor temperature
vcgencmd measure_temp

# If overheating, add cooling or reduce resolution
```

### Import Errors
```bash
# Make sure Picamera2 is installed
sudo apt install python3-picamera2

# Or install via pip
pip3 install picamera2

# Check Python version (3.9+ recommended)
python3 --version
```

### Model Too Large
```bash
# Use YOLOv8n (nano) instead of larger models
# Or quantize your model to INT8

# Export to TFLite
from ultralytics import YOLO
model = YOLO('model/best.pt')
model.export(format='tflite', int8=True)
```

## Performance Benchmarks

### Raspberry Pi 4 (4GB)
- Resolution: 640×480
- YOLOv8n: ~10 FPS
- YOLOv8s: ~5 FPS
- YOLOv8m: ~2 FPS

### Raspberry Pi 5
- Resolution: 1280×720
- YOLOv8n: ~15-20 FPS
- YOLOv8s: ~8-10 FPS
- YOLOv8m: ~4-5 FPS

### With Coral USB Accelerator
- Can achieve 30+ FPS with optimized models

## Remote Access

### Enable SSH
```bash
sudo raspi-config
# Interface Options → SSH → Enable

# Access from PC:
ssh pi@raspberrypi.local
```

### Enable VNC (Desktop Access)
```bash
sudo raspi-config
# Interface Options → VNC → Enable

# Use VNC Viewer from PC
# Connect to: raspberrypi.local
```

### View Detection Stream Remotely
```bash
# Install Flask for web streaming
pip3 install flask

# Use web streaming script (create separate script for this)
```

## Power Considerations

### For Field Operation
- Use 20,000mAh power bank (5V/3A output)
- Expected runtime: 6-8 hours
- Monitor voltage: `vcgencmd get_throttled`

### For Robot Integration
- Use 12V→5V buck converter (3A+)
- Share power with robot system
- Add capacitors for stability

## Integration with Robot Controller

```python
# In your robot controller script
from raspi_camera_detection import detect_cotton

# Get detection results
detections = detect_cotton()
for det in detections:
    x, y, confidence, class_name = det
    if confidence > 0.75 and 'ready' in class_name:
        move_gripper_to(x, y)
        pick_cotton()
```

## Next Steps

1. **Test camera**: `libcamera-hello`
2. **Run detection**: `python3 raspi_camera_detection.py`
3. **Optimize settings**: Adjust resolution/confidence
4. **Mount camera**: Use 3D printed mounts
5. **Integrate with robot**: Connect to picking controller

## Comparison: Jetson Nano vs Raspberry Pi

| Feature | Jetson Nano | Raspberry Pi 4/5 |
|---------|------------|------------------|
| AI Performance | Better (GPU) | Good (CPU) |
| Power Usage | Higher (10W) | Lower (5-8W) |
| Cost | ~$100 | ~$60 |
| Camera Support | CSI + USB | CSI + USB |
| Best For | Real-time AI | General purpose |

**Recommendation**: 
- **Jetson Nano**: Best for real-time detection with high FPS
- **Raspberry Pi**: Good for portable, battery-powered operation
