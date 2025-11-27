"""
Integrated Picking Controller for Cotton Picking Robot
Orchestrates stereo vision, YOLO detection, kinematics, and Arduino control
Main execution pipeline for autonomous cotton picking
"""

import cv2
import numpy as np
import serial
import time
from ultralytics import YOLO
from pathlib import Path
import os

from stereo_vision import StereoVisionSystem, visualize_depth_map
from continuum_kinematics import ContinuumKinematics

class PickingController:
    """
    Main controller for autonomous cotton picking
    Integrates all subsystems
    """
    
    def __init__(self,
                 model_path: str,
                 calibration_file: str = None,
                 arduino_port: str = 'COM3',
                 arduino_baud: int = 115200):
        """
        Initialize picking controller
        
        Args:
            model_path: Path to YOLO model (best.pt)
            calibration_file: Path to stereo calibration JSON
            arduino_port: Serial port for Arduino
            arduino_baud: Baud rate for serial communication
        """
        print("="*60)
        print("COTTON PICKING ROBOT - INITIALIZING")
        print("="*60)
        
        # Load YOLO model
        print(f"\nLoading YOLO model: {model_path}")
        self.yolo_model = YOLO(model_path)
        print("✓ YOLO model loaded")
        
        # Initialize stereo vision
        print("\nInitializing stereo vision system...")
        self.stereo = StereoVisionSystem(calibration_file)
        
        # Initialize kinematics
        print("\nInitializing continuum kinematics...")
        self.kinematics = ContinuumKinematics(
            num_sections=1,
            section_length=150.0,  # 150mm arm
            backbone_radius=5.0,   # 5mm from center to tendon
            servo_drum_radius=10.0  # 10mm drum
        )
        
        # Initialize serial communication with Arduino
        print(f"\nConnecting to Arduino on {arduino_port}...")
        try:
            self.arduino = serial.Serial(arduino_port, arduino_baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            
            # Read Arduino startup message
            response = self.arduino.readline().decode('utf-8', errors='ignore').strip()
            print(f"Arduino: {response}")
            print("✓ Arduino connected")
        except serial.SerialException as e:
            print(f"⚠ Warning: Could not connect to Arduino: {e}")
            print("  Continuing in simulation mode (no hardware control)")
            self.arduino = None
        
        # Detection parameters
        self.confidence_threshold = 0.5
        self.min_picking_distance = 100.0  # mm
        self.max_picking_distance = 200.0  # mm
        
        # State
        self.is_running = False
        self.picked_count = 0
        
        print("\n" + "="*60)
        print("INITIALIZATION COMPLETE")
        print("="*60)
    
    def setup_cameras(self, left_idx: int = 0, right_idx: int = 1):
        """
        Setup stereo cameras
        
        Args:
            left_idx: Left camera index
            right_idx: Right camera index
        """
        return self.stereo.setup_cameras(left_idx, right_idx)
    
    def detect_cotton(self, frame: np.ndarray) -> list:
        """
        Detect cotton in frame using YOLO
        
        Args:
            frame: Input image
        
        Returns:
            List of detections: [(class, confidence, bbox, center), ...]
        """
        results = self.yolo_model(frame, conf=self.confidence_threshold, verbose=False)
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = result.names[cls]
                
                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox = (int(x1), int(y1), int(x2), int(y2))
                
                # Calculate center
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                center = (center_x, center_y)
                
                detections.append({
                    'class': class_name,
                    'confidence': conf,
                    'bbox': bbox,
                    'center': center
                })
        
        return detections
    
    def get_3d_position(self, center_2d: tuple, depth_map: np.ndarray) -> np.ndarray:
        """
        Convert 2D detection to 3D position
        
        Args:
            center_2d: (x, y) pixel coordinates
            depth_map: Depth map from stereo
        
        Returns:
            3D position [x, y, z] in mm, or None if invalid
        """
        u, v = center_2d
        return self.stereo.get_3d_point(u, v, depth_map)
    
    def move_to_position(self, target_3d: np.ndarray, gripper_open: bool = True) -> bool:
        """
        Move arm to target 3D position
        
        Args:
            target_3d: Target position [x, y, z] in mm
            gripper_open: Gripper state
        
        Returns:
            True if successful
        """
        # Check if position is reachable
        if not self.kinematics.is_in_workspace(target_3d):
            print(f"⚠ Target out of workspace: {target_3d}")
            return False
        
        # Inverse kinematics
        tendon_lengths = self.kinematics.inverse_kinematics(target_3d)
        
        if tendon_lengths is None:
            print(f"⚠ No IK solution for target: {target_3d}")
            return False
        
        # Convert to servo angles
        servo_angles = self.kinematics.tendon_to_servo_angles(tendon_lengths)
        
        # Add gripper angle
        gripper_angle = 0 if gripper_open else 180
        
        # Send command to Arduino
        return self.send_servo_command(servo_angles, gripper_angle)
    
    def send_servo_command(self, arm_angles: np.ndarray, gripper_angle: int) -> bool:
        """
        Send servo command to Arduino
        
        Args:
            arm_angles: 3 servo angles for arm [deg]
            gripper_angle: Gripper angle [deg]
        
        Returns:
            True if successful
        """
        if self.arduino is None:
            print(f"[SIM] Servo command: S1:{int(arm_angles[0])}, S2:{int(arm_angles[1])}, "
                  f"S3:{int(arm_angles[2])}, S4:{gripper_angle}")
            return True
        
        # Format command: S1:90,S2:45,S3:120,S4:0
        cmd = f"S1:{int(arm_angles[0])},S2:{int(arm_angles[1])},S3:{int(arm_angles[2])},S4:{gripper_angle}\n"
        
        try:
            self.arduino.write(cmd.encode())
            
            # Wait for acknowledgment
            response = self.arduino.readline().decode('utf-8', errors='ignore').strip()
            
            if "OK" in response:
                return True
            else:
                print(f"Arduino error: {response}")
                return False
                
        except Exception as e:
            print(f"Serial communication error: {e}")
            return False
    
    def pick_cotton(self, position_3d: np.ndarray) -> bool:
        """
        Execute picking sequence
        
        Args:
            position_3d: Cotton position [x, y, z] in mm
        
        Returns:
            True if successful
        """
        print(f"\n→ Picking cotton at [{position_3d[0]:.1f}, {position_3d[1]:.1f}, {position_3d[2]:.1f}] mm")
        
        # Step 1: Open gripper
        print("  1. Opening gripper...")
        if not self.move_to_position(position_3d, gripper_open=True):
            print("  ✗ Failed to reach position")
            return False
        time.sleep(0.5)
        
        # Step 2: Close gripper
        print("  2. Closing gripper...")
        if not self.move_to_position(position_3d, gripper_open=False):
            print("  ✗ Failed to close gripper")
            return False
        time.sleep(0.5)
        
        # Step 3: Retract to home position
        print("  3. Retracting...")
        home_position = np.array([0.0, 0.0, 150.0])  # Straight up
        if not self.move_to_position(home_position, gripper_open=False):
            print("  ✗ Failed to retract")
            return False
        time.sleep(0.5)
        
        # Step 4: Open gripper (release)
        print("  4. Releasing...")
        if not self.move_to_position(home_position, gripper_open=True):
            print("  ✗ Failed to release")
            return False
        
        print("  ✓ Pick complete!")
        self.picked_count += 1
        return True
    
    def run_picking_pipeline(self, show_visualization: bool = True):
        """
        Run main picking pipeline loop
        
        Args:
            show_visualization: Show live camera feed with detections
        """
        print("\n" + "="*60)
        print("STARTING PICKING PIPELINE")
        print("="*60)
        print("\nControls:")
        print("  'q' - Quit")
        print("  'p' - Pause/Resume")
        print("  'd' - Toggle depth visualization")
        print("  's' - Save current frame")
        print("="*60 + "\n")
        
        self.is_running = True
        paused = False
        show_depth = False
        
        while self.is_running:
            if not paused:
                # Capture stereo frames
                left_frame, right_frame = self.stereo.capture_frames()
                
                if left_frame is None or right_frame is None:
                    print("⚠ Failed to capture frames")
                    continue
                
                # Detect cotton in left frame
                detections = self.detect_cotton(left_frame)
                
                # Filter for ready cotton
                ready_cotton = [d for d in detections if d['class'] == 'cotton_ready']
                
                if len(ready_cotton) > 0:
                    print(f"\n✓ Found {len(ready_cotton)} ready cotton")
                    
                    # Compute depth map
                    depth_map = self.stereo.compute_depth_map(left_frame, right_frame, rectify=True)
                    
                    # Process each detection
                    for i, detection in enumerate(ready_cotton):
                        # Get 3D position
                        position_3d = self.get_3d_position(detection['center'], depth_map)
                        
                        if position_3d is not None:
                            distance = position_3d[2]  # Z coordinate
                            
                            print(f"  Cotton {i+1}: {detection['class']} "
                                  f"(conf: {detection['confidence']:.2f}) "
                                  f"at {distance:.1f} mm")
                            
                            # Check if in picking range
                            if self.min_picking_distance <= distance <= self.max_picking_distance:
                                # Attempt pick
                                if self.pick_cotton(position_3d):
                                    print(f"  ✓ Successfully picked! Total: {self.picked_count}")
                                else:
                                    print(f"  ✗ Pick failed")
                                
                                # Wait before next pick
                                time.sleep(1.0)
                            else:
                                print(f"  ⚠ Out of range ({self.min_picking_distance}-{self.max_picking_distance} mm)")
                
                # Visualization
                if show_visualization:
                    # Draw detections on frame
                    vis_frame = left_frame.copy()
                    
                    for det in detections:
                        x1, y1, x2, y2 = det['bbox']
                        color = (0, 255, 0) if det['class'] == 'cotton_ready' else (0, 0, 255)
                        
                        cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, 2)
                        
                        label = f"{det['class']}: {det['confidence']:.2f}"
                        cv2.putText(vis_frame, label, (x1, y1 - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        
                        # Draw center point
                        cv2.circle(vis_frame, det['center'], 5, color, -1)
                    
                    # Add info overlay
                    cv2.putText(vis_frame, f"Picked: {self.picked_count}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(vis_frame, f"Ready: {len(ready_cotton)}", (10, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    cv2.imshow('Cotton Detection', vis_frame)
                    
                    # Show depth map if enabled
                    if show_depth and 'depth_map' in locals():
                        depth_viz = visualize_depth_map(depth_map)
                        cv2.imshow('Depth Map', depth_viz)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nStopping...")
                self.is_running = False
            elif key == ord('p'):
                paused = not paused
                status = "PAUSED" if paused else "RUNNING"
                print(f"\n{status}")
            elif key == ord('d'):
                show_depth = not show_depth
                if not show_depth:
                    cv2.destroyWindow('Depth Map')
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"capture_{timestamp}.jpg", left_frame)
                print(f"✓ Saved frame: capture_{timestamp}.jpg")
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Release resources"""
        print("\nCleaning up...")
        
        self.stereo.release()
        
        if self.arduino is not None:
            self.arduino.close()
            print("✓ Arduino connection closed")
        
        cv2.destroyAllWindows()
        
        print(f"\nSession summary:")
        print(f"  Cotton picked: {self.picked_count}")
        print("="*60)


# Main execution
if __name__ == "__main__":
    # Configuration
    MODEL_PATH = r"model\best.pt"
    CALIBRATION_FILE = r"calibration_data\stereo_calibration.json"
    ARDUINO_PORT = "COM3"  # Change to your Arduino port
    
    # Check if files exist
    if not os.path.exists(MODEL_PATH):
        print(f"❌ Model not found: {MODEL_PATH}")
        exit(1)
    
    if not os.path.exists(CALIBRATION_FILE):
        print(f"⚠ Calibration file not found: {CALIBRATION_FILE}")
        print("  Will run without calibration (depth may be inaccurate)")
        CALIBRATION_FILE = None
    
    # Initialize controller
    controller = PickingController(
        model_path=MODEL_PATH,
        calibration_file=CALIBRATION_FILE,
        arduino_port=ARDUINO_PORT
    )
    
    # Setup cameras
    print("\nDetecting cameras...")
    cameras = controller.stereo.detect_cameras()
    
    if len(cameras['available']) >= 2:
        left_idx = cameras['available'][0]
        right_idx = cameras['available'][1]
        
        if controller.setup_cameras(left_idx, right_idx):
            # Setup stereo matcher
            controller.stereo.setup_stereo_matcher('SGBM')
            
            # Run picking pipeline
            controller.run_picking_pipeline(show_visualization=True)
        else:
            print("❌ Failed to setup cameras")
    else:
        print("❌ Need at least 2 cameras for stereo vision")
