"""
Cotton Detection for Images and Videos
Process images or video files to detect cotton
"""

from ultralytics import YOLO
import cv2
import os
from pathlib import Path
from datetime import datetime

# Configuration
MODEL_PATH = Path(__file__).parent / "model" / "best.pt"
CONFIDENCE_THRESHOLD = 0.5

# Output directory
OUTPUT_DIR = Path(__file__).parent / "output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def detect_in_image(model, image_path):
    """Detect cotton in a single image"""
    print(f"\nProcessing: {image_path}")
    
    # Run inference
    results = model(image_path, conf=CONFIDENCE_THRESHOLD)
    
    # Get detections
    detections = {"cotton_ready": 0, "cotton_not_ready": 0}
    
    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls = int(box.cls[0])
            class_name = result.names[cls]
            if class_name in detections:
                detections[class_name] += 1
    
    # Save annotated image
    annotated = results[0].plot()
    output_path = os.path.join(OUTPUT_DIR, f"detected_{Path(image_path).name}")
    cv2.imwrite(output_path, annotated)
    
    print(f"✓ Ready to Pluck: {detections['cotton_ready']}")
    print(f"✓ Not Ready: {detections['cotton_not_ready']}")
    print(f"✓ Saved to: {output_path}")
    
    return detections

def detect_in_video(model, video_path):
    """Detect cotton in a video file"""
    print(f"\nProcessing video: {video_path}")
    
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"❌ Error: Could not open video {video_path}")
        return
    
    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Setup video writer
    output_path = os.path.join(OUTPUT_DIR, f"detected_{Path(video_path).name}")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    print(f"Video: {width}x{height} @ {fps}fps, {total_frames} frames")
    
    frame_count = 0
    total_detections = {"cotton_ready": 0, "cotton_not_ready": 0}
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        frame_count += 1
        
        # Run inference
        results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
        annotated_frame = results[0].plot()
        
        # Count detections
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                class_name = result.names[cls]
                if class_name in total_detections:
                    total_detections[class_name] += 1
        
        # Write frame
        out.write(annotated_frame)
        
        # Show progress
        if frame_count % 30 == 0:
            progress = (frame_count / total_frames) * 100
            print(f"Progress: {progress:.1f}% ({frame_count}/{total_frames} frames)")
    
    cap.release()
    out.release()
    
    print(f"\n✓ Video processing complete!")
    print(f"✓ Total Ready to Pluck: {total_detections['cotton_ready']}")
    print(f"✓ Total Not Ready: {total_detections['cotton_not_ready']}")
    print(f"✓ Saved to: {output_path}")

def detect_in_folder(model, folder_path):
    """Detect cotton in all images in a folder"""
    print(f"\nProcessing folder: {folder_path}")
    
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
    image_files = []
    
    for ext in image_extensions:
        image_files.extend(Path(folder_path).glob(f"*{ext}"))
        image_files.extend(Path(folder_path).glob(f"*{ext.upper()}"))
    
    if not image_files:
        print("❌ No images found in folder")
        return
    
    print(f"Found {len(image_files)} images")
    
    total_detections = {"cotton_ready": 0, "cotton_not_ready": 0}
    
    for i, img_path in enumerate(image_files, 1):
        print(f"\n[{i}/{len(image_files)}]", end=" ")
        detections = detect_in_image(model, str(img_path))
        
        for key in detections:
            total_detections[key] += detections[key]
    
    print(f"\n{'='*60}")
    print("FOLDER PROCESSING COMPLETE")
    print(f"  Total Images: {len(image_files)}")
    print(f"  Total Ready to Pluck: {total_detections['cotton_ready']}")
    print(f"  Total Not Ready: {total_detections['cotton_not_ready']}")
    print(f"{'='*60}")

def main():
    print("="*60)
    print("Cotton Detection - Image/Video Processing")
    print("="*60)
    
    # Load model
    print(f"\nLoading model from: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("✓ Model loaded successfully!")
    
    print("\n" + "="*60)
    print("SELECT MODE:")
    print("  1. Process single image")
    print("  2. Process video file")
    print("  3. Process folder of images")
    print("="*60)
    
    choice = input("\nEnter your choice (1/2/3): ").strip()
    
    if choice == "1":
        image_path = input("Enter image path: ").strip().strip('"')
        if os.path.exists(image_path):
            detect_in_image(model, image_path)
        else:
            print("❌ Image file not found!")
    
    elif choice == "2":
        video_path = input("Enter video path: ").strip().strip('"')
        if os.path.exists(video_path):
            detect_in_video(model, video_path)
        else:
            print("❌ Video file not found!")
    
    elif choice == "3":
        folder_path = input("Enter folder path: ").strip().strip('"')
        if os.path.exists(folder_path):
            detect_in_folder(model, folder_path)
        else:
            print("❌ Folder not found!")
    
    else:
        print("❌ Invalid choice!")
    
    print("\n✓ Done!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        import traceback
        traceback.print_exc()
