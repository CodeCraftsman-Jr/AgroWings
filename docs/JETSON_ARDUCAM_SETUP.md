# Arducam + Jetson Nano Setup for Cotton Detection

## Hardware Requirements
- NVIDIA Jetson Nano (4GB recommended)
- Arducam camera module (IMX219 or IMX477)
- CSI camera cable
- Power supply (5V/4A for Jetson Nano with camera)

## Arducam Installation on Jetson Nano

### 1. Physical Connection
```bash
# Power off Jetson Nano
sudo shutdown -h now

# Connect Arducam to CSI camera port (CAMERA 0 or CAMERA 1)
# Blue side of ribbon cable faces away from heatsink
# Power on Jetson Nano
```

### 2. Verify Camera Detection
```bash
# Check if camera is detected
ls /dev/video*
# Should show: /dev/video0

# Test camera with NVIDIA utility
nvgstcapture-1.0
# Press 'j' to capture image, 'q' to quit

# Or use GStreamer test
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! nvegltransform ! nveglglessink
```

### 3. Install Dependencies
```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install Python dependencies
pip3 install opencv-python
pip3 install ultralytics

# For hardware acceleration (optional)
pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu116
```

### 4. Camera Configuration
Edit the camera settings in `jetson_arducam_detection.py`:

```python
# For IMX219 (8MP camera)
CAMERA_WIDTH = 1280   # or 1920, 3280
CAMERA_HEIGHT = 720   # or 1080, 2464

# For IMX477 (12MP camera)
CAMERA_WIDTH = 1280   # or 1920, 4056
CAMERA_HEIGHT = 720   # or 1080, 3040

# Rotation if camera is mounted upside down
CAMERA_FLIP = 2  # 180 degrees
```

## Physical Mounting Options

### Option 1: Standard CSI Mount
- Use Arducam's official CSI camera holder
- Attach to Jetson Nano case or custom mount
- Position camera for optimal cotton view

### Option 2: Custom 3D Printed Mount
Download STL files from:
- Thingiverse: "Jetson Nano camera mount"
- https://www.thingiverse.com/thing:3518410

### Option 3: Adjustable Arm Mount
- Use flexible gooseneck mount (1/4" thread)
- Arducam UC-391: Camera with adjustable mount
- Attach to robot arm or fixed position

### Option 4: Robot Gripper Mount
For integration with picking robot:
```
Camera Position:
- Mount above or beside gripper
- 15-30cm distance from cotton
- Angle: 30-45 degrees downward
- Ensure clear view without gripper obstruction
```

## Running on Jetson Nano

### Copy files to Jetson
```bash
# On your PC, transfer files
scp -r AgroWings/ jetson@<jetson-ip>:~/

# Or use USB drive
# Or use git clone
```

### Run the detection script
```bash
cd ~/AgroWings
python3 jetson_arducam_detection.py
```

## Troubleshooting

### Camera not detected
```bash
# Restart camera daemon
sudo systemctl restart nvargus-daemon

# Check camera module
dmesg | grep imx219  # or imx477

# Verify GStreamer plugins
gst-inspect-1.0 nvarguscamerasrc
```

### Low FPS / Performance
```bash
# Set Jetson to max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Check GPU usage
tegrastats
```

### Image quality issues
- Adjust focus ring on camera lens
- Check lighting conditions
- Modify exposure in GStreamer pipeline

## Integration with Robot Controller

Connect to `robot_interface.py` and `picking_controller.py`:

```python
# In picking_controller.py
from jetson_arducam_detection import detect_cotton

# Get cotton coordinates
detections = detect_cotton(frame)
for detection in detections:
    x, y, confidence = detection
    if confidence > 0.75:
        move_robot_to(x, y)
        pick_cotton()
```

## Performance Tips
- Use NVIDIA TensorRT for model optimization
- Enable hardware acceleration (CUDA)
- Reduce resolution if FPS is low
- Use INT8 quantization for faster inference
