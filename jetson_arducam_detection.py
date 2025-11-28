"""
Cotton Detection using Arducam on NVIDIA Jetson Nano
Real-time cotton detection with hardware-accelerated inference
"""

from ultralytics import YOLO
import cv2
from pathlib import Path
from datetime import datetime
import os

# Configuration
MODEL_PATH = Path(__file__).parent / "model" / "best.pt"
CONFIDENCE_THRESHOLD = 0.75  # Only show detections above 75% confidence

# Arducam Configuration for Jetson Nano
# Common Arducam configurations:
# - IMX219: 3280x2464, 1920x1080, 1640x1232, 1280x720
# - IMX477: 4056x3040, 1920x1080, 1280x720
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
CAMERA_FLIP = 0  # 0=no rotation, 2=180 degrees

# Output directory
OUTPUT_DIR = Path(__file__).parent / "jetson_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    """
    Create GStreamer pipeline string for Jetson Nano CSI camera
    
    Args:
        sensor_id: Camera sensor ID (0 or 1 for dual camera setup)
        capture_width: Camera sensor capture width
        capture_height: Camera sensor capture height
        display_width: Output frame width
        display_height: Output frame height
        framerate: Camera framerate
        flip_method: Image rotation (0=none, 1=90ccw, 2=180, 3=90cw, 4=horizontal, 5=upper-right, 6=vertical, 7=upper-left)
    
    Returns:
        GStreamer pipeline string
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )

print("=" * 60)
print("Cotton Detection - Jetson Nano + Arducam")
print("=" * 60)
print("\nHardware Setup:")
print("  - NVIDIA Jetson Nano")
print("  - Arducam (IMX219/IMX477) connected to CSI port")
print("\nConfiguration:")
print(f"  - Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
print(f"  - FPS: {CAMERA_FPS}")
print(f"  - Confidence Threshold: {CONFIDENCE_THRESHOLD*100}%")
print("=" * 60)

# Load YOLO model
print(f"\nLoading model from: {MODEL_PATH}")
try:
    model = YOLO(str(MODEL_PATH))
    print("✓ Model loaded successfully!")
except Exception as e:
    print(f"✗ Error loading model: {e}")
    exit(1)

# Try GStreamer pipeline first (for CSI camera)
print("\nAttempting to open Arducam via GStreamer (CSI)...")
pipeline = gstreamer_pipeline(
    sensor_id=0,
    capture_width=CAMERA_WIDTH,
    capture_height=CAMERA_HEIGHT,
    display_width=CAMERA_WIDTH,
    display_height=CAMERA_HEIGHT,
    framerate=CAMERA_FPS,
    flip_method=CAMERA_FLIP,
)
print(f"Pipeline: {pipeline}")

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# Fallback to USB camera if GStreamer fails
if not cap.isOpened():
    print("✗ GStreamer failed, trying USB camera mode...")
    cap = cv2.VideoCapture(0)
    
if not cap.isOpened():
    print("✗ Failed to open camera!")
    print("\nTroubleshooting:")
    print("  1. Check if Arducam is properly connected to CSI port")
    print("  2. Verify camera with: 'nvgstcapture-1.0'")
    print("  3. Check if camera is detected: 'ls /dev/video*'")
    print("  4. Ensure no other process is using the camera")
    print("  5. Try: 'sudo systemctl restart nvargus-daemon'")
    exit(1)

print("✓ Camera opened successfully!")
print("\n" + "=" * 60)
print("CONTROLS:")
print("  's' - Save current frame")
print("  'q' - Quit")
print("  'f' - Toggle fullscreen")
print("=" * 60)

frame_count = 0
saved_count = 0
fullscreen = False

try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("✗ Failed to grab frame")
            break
        
        frame_count += 1
        
        # Run detection
        results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
        
        # Draw results on frame
        annotated_frame = results[0].plot()
        
        # Count detections
        detections = results[0].boxes
        cotton_ready_count = 0
        cotton_not_ready_count = 0
        
        for box in detections:
            cls = int(box.cls[0])
            class_name = model.names[cls]
            if 'ready' in class_name.lower():
                cotton_ready_count += 1
            else:
                cotton_not_ready_count += 1
        
        # Add detection info to frame
        info_text = f"Ready: {cotton_ready_count} | Not Ready: {cotton_not_ready_count} | FPS: {CAMERA_FPS}"
        cv2.putText(annotated_frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Show frame
        window_name = 'Jetson Arducam Cotton Detection'
        cv2.imshow(window_name, annotated_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('s'):
            # Save frame
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"jetson_capture_{timestamp}.jpg"
            filepath = OUTPUT_DIR / filename
            cv2.imwrite(str(filepath), annotated_frame)
            saved_count += 1
            print(f"✓ Saved: {filename} (Ready: {cotton_ready_count}, Not Ready: {cotton_not_ready_count})")
        elif key == ord('f'):
            # Toggle fullscreen
            fullscreen = not fullscreen
            if fullscreen:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

except KeyboardInterrupt:
    print("\n\nInterrupted by user")

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n" + "=" * 60)
    print("SESSION SUMMARY")
    print("=" * 60)
    print(f"Frames processed: {frame_count}")
    print(f"Images saved: {saved_count}")
    print(f"Output directory: {OUTPUT_DIR}")
    print("=" * 60)
