"""
Cotton Detection using DroidCam
Real-time cotton detection using phone camera via DroidCam
"""

from ultralytics import YOLO
import cv2
from pathlib import Path
from datetime import datetime
import os

# Configuration
MODEL_PATH = Path(__file__).parent / "model" / "best.pt"
CONFIDENCE_THRESHOLD = 0.75  # Only show detections above 75% confidence

# DroidCam camera index - DroidCam Client creates a virtual webcam
# Try 0, 1, 2, or 3 to find the DroidCam virtual camera
CAMERA_INDEX = 0  # Change this if DroidCam isn't on index 0

# Output directory
OUTPUT_DIR = Path(__file__).parent / "droidcam_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("=" * 60)
print("Cotton Detection - DroidCam Client Mode")
print("=" * 60)
print("\nSetup Instructions:")
print("1. Start DroidCam Client on PC")
print("2. Connect to your phone via WiFi or USB")
print("3. Make sure video is showing in DroidCam Client")
print("4. Run this script")
print("\nCurrent Camera Index:", CAMERA_INDEX)
print("=" * 60)

# Load YOLO model
print(f"\nLoading model from: {MODEL_PATH}")
try:
    model = YOLO(str(MODEL_PATH))
    print("✓ Model loaded successfully!")
except Exception as e:
    print(f"✗ Error loading model: {e}")
    exit(1)

# Connect to DroidCam
print(f"\nConnecting to camera index: {CAMERA_INDEX}")
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print("✗ Failed to connect to DroidCam!")
    print("\nTroubleshooting:")
    print("  1. Make sure DroidCam Client is running and connected")
    print("  2. Check if video is showing in DroidCam Client window")
    print("  3. Try different camera indices (0, 1, 2, 3)")
    print("  4. Close other apps that might be using the camera")
    print("\nTip: Run test_camera.py to see available camera indices")
    exit(1)

print("✓ Connected to DroidCam!")
print("\n" + "=" * 60)
print("CONTROLS:")
print("  's' - Save current frame")
print("  'q' - Quit")
print("=" * 60)

frame_count = 0
saved_count = 0

try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("✗ Failed to grab frame from DroidCam")
            break
        
        frame_count += 1
        
        # Run detection every frame
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
        
        # Show frame
        cv2.imshow('DroidCam Cotton Detection', annotated_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('s'):
            # Save frame
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"droidcam_capture_{timestamp}.jpg"
            filepath = OUTPUT_DIR / filename
            cv2.imwrite(str(filepath), annotated_frame)
            saved_count += 1
            print(f"✓ Saved: {filename} (Ready: {cotton_ready_count}, Not Ready: {cotton_not_ready_count})")

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
