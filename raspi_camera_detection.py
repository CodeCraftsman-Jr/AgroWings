"""
Cotton Detection using Raspberry Pi Camera
Real-time cotton detection optimized for Raspberry Pi
"""

from ultralytics import YOLO
import cv2
from pathlib import Path
from datetime import datetime
import os
from picamera2 import Picamera2
import numpy as np

# Configuration
MODEL_PATH = Path(__file__).parent / "model" / "best.pt"
CONFIDENCE_THRESHOLD = 0.75  # Only show detections above 75% confidence

# Camera Configuration for Raspberry Pi
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

# Output directory
OUTPUT_DIR = Path(__file__).parent / "raspi_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("=" * 60)
print("Cotton Detection - Raspberry Pi Camera")
print("=" * 60)
print("\nHardware Setup:")
print("  - Raspberry Pi 4/5 (4GB+ recommended)")
print("  - Pi Camera Module 2/3 or HQ Camera")
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

# Initialize Picamera2
print("\nInitializing Pi Camera...")
try:
    picam2 = Picamera2()
    
    # Configure camera
    camera_config = picam2.create_preview_configuration(
        main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": CAMERA_FPS}
    )
    picam2.configure(camera_config)
    picam2.start()
    
    print("✓ Camera initialized successfully!")
except Exception as e:
    print(f"✗ Failed to initialize camera: {e}")
    print("\nTroubleshooting:")
    print("  1. Check if camera is connected properly")
    print("  2. Enable camera: sudo raspi-config → Interface Options → Camera")
    print("  3. Verify camera: libcamera-hello")
    print("  4. Install Picamera2: sudo apt install python3-picamera2")
    exit(1)

print("\n" + "=" * 60)
print("CONTROLS:")
print("  's' - Save current frame")
print("  'q' - Quit")
print("  'p' - Pause/Resume detection")
print("=" * 60)

frame_count = 0
saved_count = 0
paused = False

try:
    while True:
        # Capture frame
        frame = picam2.capture_array()
        
        # Convert from RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame_count += 1
        
        if not paused:
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
            info_text = f"Ready: {cotton_ready_count} | Not Ready: {cotton_not_ready_count}"
            cv2.putText(annotated_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            annotated_frame = frame.copy()
            cv2.putText(annotated_frame, "PAUSED", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Show frame
        cv2.imshow('Raspberry Pi Cotton Detection', annotated_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('s'):
            # Save frame
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"raspi_capture_{timestamp}.jpg"
            filepath = OUTPUT_DIR / filename
            cv2.imwrite(str(filepath), annotated_frame)
            saved_count += 1
            if not paused:
                print(f"✓ Saved: {filename} (Ready: {cotton_ready_count}, Not Ready: {cotton_not_ready_count})")
            else:
                print(f"✓ Saved: {filename}")
        elif key == ord('p'):
            paused = not paused
            status = "PAUSED" if paused else "RESUMED"
            print(f"Detection {status}")

except KeyboardInterrupt:
    print("\n\nInterrupted by user")

finally:
    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
    
    print("\n" + "=" * 60)
    print("SESSION SUMMARY")
    print("=" * 60)
    print(f"Frames processed: {frame_count}")
    print(f"Images saved: {saved_count}")
    print(f"Output directory: {OUTPUT_DIR}")
    print("=" * 60)
