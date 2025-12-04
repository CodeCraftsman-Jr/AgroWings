"""
Monocular Camera + ToF Depth Fusion for 3D Detection
Replaces stereo vision with single camera + VL53L5CX depth sensor
"""

import cv2
import numpy as np
import time
from picamera2 import Picamera2
from ultralytics import YOLO

from tof_sensor import ToFDepthSensor, MockToFDepthSensor


class MonoToFVision:
    """
    Monocular camera with Time-of-Flight depth sensing
    
    Combines:
    - Raspberry Pi Camera Module (2D detections via YOLO)
    - VL53L5CX ToF sensor (depth measurements)
    
    Outputs 3D coordinates of detected cotton bolls
    """
    
    def __init__(self, 
                 model_path='model/best.pt',
                 camera_resolution=(640, 480),
                 camera_fps=30,
                 use_mock_tof=False):
        """
        Initialize monocular vision system
        
        Args:
            model_path (str): Path to YOLO model
            camera_resolution (tuple): Camera resolution (width, height)
            camera_fps (int): Camera frame rate
            use_mock_tof (bool): Use mock ToF sensor for testing
        """
        
        self.camera_resolution = camera_resolution
        self.camera_fps = camera_fps
        
        print("Initializing Mono+ToF Vision System...")
        
        # Initialize camera
        print("Setting up Raspberry Pi camera...")
        self.camera = Picamera2()
        
        # Configure camera
        camera_config = self.camera.create_preview_configuration(
            main={"size": camera_resolution, "format": "RGB888"}
        )
        self.camera.configure(camera_config)
        self.camera.start()
        
        # Allow camera to warm up
        time.sleep(2)
        print(f"Camera ready: {camera_resolution[0]}x{camera_resolution[1]} @ {camera_fps}fps")
        
        # Initialize YOLO model
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.confidence_threshold = 0.5
        print("YOLO model loaded")
        
        # Initialize ToF sensor
        print("Initializing VL53L5CX ToF sensor...")
        try:
            if use_mock_tof:
                raise ImportError("Using mock sensor")
            self.tof_sensor = ToFDepthSensor(ranging_freq=15, resolution=8)
            print("Real ToF sensor initialized")
        except:
            print("Warning: Real ToF sensor not available, using mock sensor")
            self.tof_sensor = MockToFDepthSensor(ranging_freq=15, resolution=8)
        
        # Camera intrinsic parameters (calibration needed for accurate 3D)
        # These are typical values - should be calibrated for your specific camera
        self.focal_length_x = 500  # pixels (approximate for RPi Camera)
        self.focal_length_y = 500  # pixels
        self.principal_point_x = camera_resolution[0] / 2
        self.principal_point_y = camera_resolution[1] / 2
        
        # Camera intrinsic matrix
        self.K = np.array([
            [self.focal_length_x, 0, self.principal_point_x],
            [0, self.focal_length_y, self.principal_point_y],
            [0, 0, 1]
        ])
        
        print("Mono+ToF Vision System ready!")
    
    def capture_frame(self):
        """
        Capture a frame from the camera
        
        Returns:
            numpy.ndarray: RGB image
        """
        frame = self.camera.capture_array()
        return frame
    
    def detect_cotton(self, frame, confidence=None):
        """
        Detect cotton in frame using YOLO
        
        Args:
            frame (numpy.ndarray): Input image
            confidence (float): Confidence threshold (default: self.confidence_threshold)
            
        Returns:
            list: List of detections with format:
                  [{'bbox': (x1, y1, x2, y2), 'confidence': float, 'class': str}]
        """
        if confidence is None:
            confidence = self.confidence_threshold
        
        # Run YOLO inference
        results = self.model(frame, conf=confidence, verbose=False)
        
        detections = []
        
        if len(results) > 0:
            result = results[0]
            
            if result.boxes is not None and len(result.boxes) > 0:
                for box in result.boxes:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())
                    class_name = result.names[cls]
                    
                    detections.append({
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'confidence': conf,
                        'class': class_name,
                        'class_id': cls
                    })
        
        return detections
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth_mm):
        """
        Convert 2D pixel + depth to 3D coordinates
        
        Args:
            pixel_x (float): X coordinate in pixels
            pixel_y (float): Y coordinate in pixels
            depth_mm (float): Depth in millimeters
            
        Returns:
            numpy.ndarray: 3D coordinates (X, Y, Z) in millimeters
        """
        if depth_mm is None or depth_mm <= 0:
            return None
        
        # Convert depth from mm to same units (mm)
        Z = depth_mm
        
        # Apply inverse camera projection
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        X = (pixel_x - self.principal_point_x) * Z / self.focal_length_x
        Y = (pixel_y - self.principal_point_y) * Z / self.focal_length_y
        
        return np.array([X, Y, Z])
    
    def get_3d_detections(self, frame=None, confidence=None):
        """
        Get 3D coordinates of detected cotton
        
        Args:
            frame (numpy.ndarray): Input frame (if None, captures new frame)
            confidence (float): Detection confidence threshold
            
        Returns:
            list: List of 3D detections with format:
                  [{'bbox': (x1,y1,x2,y2), 'confidence': float, 'class': str,
                    'center_2d': (x, y), 'depth_mm': float, 'position_3d': (X,Y,Z)}]
        """
        # Capture frame if not provided
        if frame is None:
            frame = self.capture_frame()
        
        # Detect cotton in 2D
        detections_2d = self.detect_cotton(frame, confidence)
        
        # Add 3D information
        detections_3d = []
        
        for detection in detections_2d:
            bbox = detection['bbox']
            x1, y1, x2, y2 = bbox
            
            # Calculate center of bounding box
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Get depth from ToF sensor at detection center
            depth_mm = self.tof_sensor.get_average_depth_in_bbox(
                bbox, 
                self.camera_resolution[0], 
                self.camera_resolution[1]
            )
            
            # Convert to 3D coordinates
            if depth_mm is not None:
                position_3d = self.pixel_to_3d(center_x, center_y, depth_mm)
            else:
                position_3d = None
            
            # Add to results
            detection_3d = detection.copy()
            detection_3d['center_2d'] = (int(center_x), int(center_y))
            detection_3d['depth_mm'] = depth_mm
            detection_3d['position_3d'] = position_3d
            
            detections_3d.append(detection_3d)
        
        return detections_3d
    
    def visualize_detections(self, frame, detections_3d):
        """
        Draw detections on frame with 3D information
        
        Args:
            frame (numpy.ndarray): Input frame
            detections_3d (list): 3D detections from get_3d_detections()
            
        Returns:
            numpy.ndarray: Annotated frame
        """
        annotated_frame = frame.copy()
        
        for detection in detections_3d:
            bbox = detection['bbox']
            x1, y1, x2, y2 = bbox
            confidence = detection['confidence']
            class_name = detection['class']
            center_2d = detection['center_2d']
            depth_mm = detection['depth_mm']
            position_3d = detection['position_3d']
            
            # Choose color based on class
            if class_name == 'cotton_ready':
                color = (0, 255, 0)  # Green
            else:
                color = (255, 165, 0)  # Orange
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            cv2.circle(annotated_frame, center_2d, 5, color, -1)
            
            # Prepare label text
            label = f"{class_name}: {confidence:.2f}"
            
            if depth_mm is not None:
                label += f" | Depth: {depth_mm:.0f}mm"
            
            if position_3d is not None:
                x, y, z = position_3d
                label2 = f"3D: ({x:.0f}, {y:.0f}, {z:.0f})"
            else:
                label2 = "3D: N/A"
            
            # Draw labels
            cv2.putText(annotated_frame, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(annotated_frame, label2, (x1, y1 - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated_frame
    
    def get_nearest_ready_cotton(self, detections_3d):
        """
        Find nearest ready cotton from 3D detections
        
        Args:
            detections_3d (list): 3D detections
            
        Returns:
            dict: Nearest ready cotton detection, or None
        """
        ready_cotton = [d for d in detections_3d 
                       if d['class'] == 'cotton_ready' and d['position_3d'] is not None]
        
        if not ready_cotton:
            return None
        
        # Find nearest (minimum Z distance)
        nearest = min(ready_cotton, key=lambda d: d['position_3d'][2])
        
        return nearest
    
    def set_camera_parameters(self, focal_length_x, focal_length_y, 
                             principal_point_x, principal_point_y):
        """
        Set calibrated camera parameters
        
        Args:
            focal_length_x (float): Focal length in X direction (pixels)
            focal_length_y (float): Focal length in Y direction (pixels)
            principal_point_x (float): Principal point X (pixels)
            principal_point_y (float): Principal point Y (pixels)
        """
        self.focal_length_x = focal_length_x
        self.focal_length_y = focal_length_y
        self.principal_point_x = principal_point_x
        self.principal_point_y = principal_point_y
        
        # Update intrinsic matrix
        self.K = np.array([
            [self.focal_length_x, 0, self.principal_point_x],
            [0, self.focal_length_y, self.principal_point_y],
            [0, 0, 1]
        ])
        
        print(f"Camera parameters updated:")
        print(f"  Focal length: ({focal_length_x:.1f}, {focal_length_y:.1f})")
        print(f"  Principal point: ({principal_point_x:.1f}, {principal_point_y:.1f})")
    
    def cleanup(self):
        """Stop camera and sensors"""
        print("Cleaning up Mono+ToF Vision System...")
        
        if self.camera:
            self.camera.stop()
            print("Camera stopped")
        
        if self.tof_sensor:
            self.tof_sensor.stop()
            print("ToF sensor stopped")


if __name__ == "__main__":
    """Test mono+ToF vision system"""
    
    print("Testing Mono+ToF Vision System...")
    
    # Initialize vision system
    vision = MonoToFVision(
        model_path='model/best.pt',
        camera_resolution=(640, 480),
        camera_fps=30,
        use_mock_tof=False  # Set to True if no real ToF sensor
    )
    
    print("\nCapturing and processing frames...")
    print("Press Ctrl+C to stop\n")
    
    try:
        frame_count = 0
        
        while True:
            # Capture and detect
            frame = vision.capture_frame()
            detections_3d = vision.get_3d_detections(frame, confidence=0.5)
            
            # Print detections
            if detections_3d:
                print(f"\n--- Frame {frame_count} ---")
                print(f"Found {len(detections_3d)} cotton boll(s):")
                
                for i, det in enumerate(detections_3d):
                    print(f"\n  Detection {i+1}:")
                    print(f"    Class: {det['class']}")
                    print(f"    Confidence: {det['confidence']:.2f}")
                    print(f"    2D Center: {det['center_2d']}")
                    print(f"    Depth: {det['depth_mm']:.0f} mm" if det['depth_mm'] else "    Depth: N/A")
                    
                    if det['position_3d'] is not None:
                        x, y, z = det['position_3d']
                        print(f"    3D Position: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
                    else:
                        print(f"    3D Position: N/A")
                
                # Find nearest ready cotton
                nearest = vision.get_nearest_ready_cotton(detections_3d)
                if nearest:
                    print(f"\n  → Nearest ready cotton at {nearest['position_3d'][2]:.0f} mm")
            
            # Visualize (save to file for remote viewing)
            if detections_3d and frame_count % 10 == 0:
                annotated = vision.visualize_detections(frame, detections_3d)
                cv2.imwrite(f'output/mono_tof_frame_{frame_count}.jpg', 
                           cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
                print(f"  Saved annotated frame to output/mono_tof_frame_{frame_count}.jpg")
            
            frame_count += 1
            time.sleep(0.5)  # 2 Hz for testing
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    finally:
        vision.cleanup()
        print("Test complete!")
