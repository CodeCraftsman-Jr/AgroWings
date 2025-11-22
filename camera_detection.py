"""
Real-time Cotton Detection using Camera
Detects cotton ready for plucking vs not ready
"""

from ultralytics import YOLO
import cv2
import numpy as np
from datetime import datetime
import os

# Configuration
MODEL_PATH = r"c:\Users\vasa\OneDrive\Desktop\Cotton Detection\model\best.pt"
CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence for detection
CAMERA_INDEX = 0  # 0 for default webcam, change if you have multiple cameras

# Create output directory for saving detections
OUTPUT_DIR = r"c:\Users\vasa\OneDrive\Desktop\Cotton Detection\detections"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def main():
    print("="*60)
    print("Cotton Detection System - Real-time Camera Detection")
    print("="*60)
    
    # Load the trained model
    print(f"\nLoading model from: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("✓ Model loaded successfully!")
    
    # Open camera
    print(f"\nOpening camera (index {CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print("❌ Error: Could not open camera!")
        print("Please check:")
        print("  - Camera is connected")
        print("  - Camera permissions are enabled")
        print("  - Try changing CAMERA_INDEX in the script")
        return
    
    print("✓ Camera opened successfully!")
    print("\n" + "="*60)
    print("CONTROLS:")
    print("  - Press 'q' to quit")
    print("  - Press 's' to save current frame")
    print("  - Press 'SPACE' to pause/resume")
    print("="*60 + "\n")
    
    # Set camera resolution (optional)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    paused = False
    frame_count = 0
    detection_counts = {"cotton_ready": 0, "cotton_not_ready": 0}
    
    while True:
        if not paused:
            ret, frame = cap.read()
            
            if not ret:
                print("❌ Failed to grab frame")
                break
            
            frame_count += 1
            
            # Run inference on the frame
            results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
            
            # Get the annotated frame
            annotated_frame = results[0].plot()
            
            # Count detections
            current_detections = {"cotton_ready": 0, "cotton_not_ready": 0}
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = result.names[cls]
                    
                    if class_name in current_detections:
                        current_detections[class_name] += 1
            
            # Update total counts
            for key in current_detections:
                detection_counts[key] += current_detections[key]
            
            # Add information overlay
            overlay_height = 120
            overlay = np.zeros((overlay_height, annotated_frame.shape[1], 3), dtype=np.uint8)
            overlay[:] = (0, 0, 0)  # Black background
            
            # Add text to overlay
            cv2.putText(overlay, "Cotton Detection System", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.putText(overlay, f"Ready to Pluck: {current_detections['cotton_ready']}", (10, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.putText(overlay, f"Not Ready: {current_detections['cotton_not_ready']}", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.putText(overlay, f"Total Detections - Ready: {detection_counts['cotton_ready']} | Not Ready: {detection_counts['cotton_not_ready']}", 
                       (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Combine overlay with frame
            final_frame = np.vstack([overlay, annotated_frame])
        else:
            # Show paused message
            pause_text = "PAUSED - Press SPACE to resume"
            cv2.putText(final_frame, pause_text, (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Display the frame
        cv2.imshow("Cotton Detection - Camera Feed", final_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\n👋 Quitting...")
            break
        elif key == ord('s'):
            # Save current frame
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(OUTPUT_DIR, f"detection_{timestamp}.jpg")
            cv2.imwrite(filename, final_frame)
            print(f"✓ Saved: {filename}")
        elif key == ord(' '):
            # Toggle pause
            paused = not paused
            if paused:
                print("⏸ Paused")
            else:
                print("▶ Resumed")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n" + "="*60)
    print("FINAL STATISTICS:")
    print(f"  Total Frames Processed: {frame_count}")
    print(f"  Cotton Ready to Pluck: {detection_counts['cotton_ready']}")
    print(f"  Cotton Not Ready: {detection_counts['cotton_not_ready']}")
    print(f"  Total Detections: {sum(detection_counts.values())}")
    print("="*60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        import traceback
        traceback.print_exc()
