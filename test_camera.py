"""
Test which camera indices are available on your system
"""

import cv2

print("="*60)
print("Camera Detection Test")
print("="*60)
print("\nTesting camera indices 0-5...")
print()

available_cameras = []

for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"✓ Camera {i}: Available ({int(width)}x{int(height)})")
            available_cameras.append(i)
        cap.release()
    else:
        print(f"✗ Camera {i}: Not available")

print("\n" + "="*60)
if available_cameras:
    print(f"Found {len(available_cameras)} camera(s): {available_cameras}")
    print(f"\nUpdate CAMERA_INDEX in camera_detection.py to: {available_cameras[0]}")
else:
    print("No cameras found!")
    print("\nPossible reasons:")
    print("  1. No camera is connected")
    print("  2. Camera is being used by another application")
    print("  3. Camera permissions are not granted")
    print("  4. Camera drivers are not installed")
    print("\nFor USB cameras:")
    print("  - Make sure the USB cable is properly connected")
    print("  - Try a different USB port")
    print("  - Check Device Manager for camera status")
print("="*60)
