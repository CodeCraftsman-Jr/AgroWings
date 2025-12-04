"""
Integrated Picking Controller for Cotton Picking Robot
Orchestrates mono+ToF vision, YOLO detection, kinematics, and ESP32 servo control
Main execution pipeline for autonomous cotton picking
"""

import cv2
import numpy as np
import serial
import time
from ultralytics import YOLO
from pathlib import Path
import os

from mono_tof_vision import MonoToFVision
from continuum_kinematics import ContinuumKinematics

class PickingController:
    """
    Main controller for autonomous cotton picking
    Integrates all subsystems: Mono+ToF vision, gripper control, kinematics, Arduino/ESP32
    """
    
    def __init__(self,
                 model_path: str,
                 arduino_port: str = 'COM3',
                 arduino_baud: int = 115200,
                 camera_resolution: tuple = (640, 480),
                 use_mock_hardware: bool = False):
        """
        Initialize picking controller
        
        Args:
            model_path: Path to YOLO model (best.pt)
            arduino_port: Serial port for Arduino/ESP32
            arduino_baud: Baud rate for serial communication
            camera_resolution: Camera resolution (width, height)
            use_mock_hardware: Use mock hardware for testing without real devices
        """
        print("="*60)
        print("COTTON PICKING ROBOT - INITIALIZING")
        print("="*60)
        
        self.use_mock_hardware = use_mock_hardware
        
        # Initialize mono+ToF vision system
        print("\nInitializing Mono+ToF vision system...")
        self.vision = MonoToFVision(
            model_path=model_path,
            camera_resolution=camera_resolution,
            camera_fps=30,
            use_mock_tof=use_mock_hardware
        )
        print("✓ Vision system ready")
        
        # Initialize kinematics
        print("\nInitializing continuum kinematics...")
        self.kinematics = ContinuumKinematics(
            num_sections=1,
            section_length=150.0,  # 150mm arm
            backbone_radius=5.0,   # 5mm from center to tendon
            servo_drum_radius=10.0  # 10mm drum
        )
        print("✓ Kinematics initialized")
        
        # Initialize Arduino serial connection
        print(f"\nInitializing Arduino controller ({arduino_port})...")
        try:
            if use_mock_hardware:
                raise serial.SerialException("Using mock hardware")
            self.arduino = serial.Serial(
                port=arduino_port,
                baudrate=arduino_baud,
                timeout=1.0
            )
            time.sleep(2)  # Wait for Arduino reset
            print("✓ Arduino controller connected")
        except serial.SerialException as e:
            print(f"⚠ Warning: Could not connect to Arduino: {e}")
            print("  Using mock mode")
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
    
    def get_3d_detections(self):
        """
        Get 3D detections from vision system
        
        Returns:
            List of 3D detections with position information
        """
        return self.vision.get_3d_detections(confidence=self.confidence_threshold)
    
    def move_to_position(self, target_3d: np.ndarray) -> bool:
        """
        Move arm to target 3D position
        
        Args:
            target_3d: Target position [x, y, z] in mm
        
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
        
        # Send command to ESP32
        return self.send_servo_command(servo_angles)
    
    def send_servo_command(self, arm_angles: np.ndarray, gripper_angle: int = None) -> bool:
        """
        Send servo command to Arduino
        
        Args:
            arm_angles: 3 servo angles for arm [deg]
            gripper_angle: Optional gripper servo angle (0-180, None = no change)
        
        Returns:
            True if successful
        """
        # Format command: S1:90,S2:45,S3:120 or S1:90,S2:45,S3:120,S4:0
        cmd = f"S1:{int(arm_angles[0])},S2:{int(arm_angles[1])},S3:{int(arm_angles[2])}"
        if gripper_angle is not None:
            cmd += f",S4:{int(gripper_angle)}"
        cmd += "\n"
        
        if self.arduino is None:
            print(f"[SIM] Servo command: {cmd.strip()}")
            return True
        
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
        Execute gripper picking sequence
        
        Args:
            position_3d: Cotton position [x, y, z] in mm
        
        Returns:
            True if successful
        """
        print(f"\n→ Picking cotton at [{position_3d[0]:.1f}, {position_3d[1]:.1f}, {position_3d[2]:.1f}] mm")
        
        # Step 1: Open gripper
        print("  1. Opening gripper...")
        tendon_lengths = self.kinematics.inverse_kinematics(position_3d)
        if tendon_lengths is None:
            print("  ✗ No IK solution")
            return False
        servo_angles = self.kinematics.tendon_to_servo_angles(tendon_lengths)
        if not self.send_servo_command(servo_angles, gripper_angle=0):  # 0 = open
            print("  ✗ Failed to open gripper")
            return False
        time.sleep(0.3)
        
        # Step 2: Move to target position
        print("  2. Moving to target...")
        if not self.move_to_position(position_3d):
            print("  ✗ Failed to reach position")
            return False
        time.sleep(0.3)
        
        # Step 3: Close gripper
        print("  3. Closing gripper...")
        if not self.send_servo_command(servo_angles, gripper_angle=180):  # 180 = closed
            print("  ✗ Failed to close gripper")
            return False
        time.sleep(0.5)  # Wait for grip
        
        # Step 4: Retract to home position
        print("  4. Retracting to home...")
        home_position = np.array([0.0, 0.0, 150.0])  # Straight up
        tendon_home = self.kinematics.inverse_kinematics(home_position)
        servo_home = self.kinematics.tendon_to_servo_angles(tendon_home)
        if not self.send_servo_command(servo_home, gripper_angle=180):  # Keep closed
            print("  ✗ Failed to retract")
            return False
        time.sleep(0.3)
        
        # Step 5: Release gripper (deposit cotton)
        print("  5. Releasing cotton...")
        if not self.send_servo_command(servo_home, gripper_angle=0):  # Open
            print("  ✗ Failed to release")
            return False
        
        time.sleep(0.2)
        
        print("  ✓ Pick complete!")
        self.picked_count += 1
        return True
    
    def run_picking_pipeline(self, show_visualization: bool = True):
        """
        Run main picking pipeline loop with mono+ToF vision and vacuum control
        
        Args:
            show_visualization: Show live camera feed with detections
        """
        print("\n" + "="*60)
        print("STARTING PICKING PIPELINE")
        print("="*60)
        print("\nControls:")
        print("  'q' - Quit")
        print("  'p' - Pause/Resume")
        print("  'g' - Test gripper")
        print("  's' - Save current frame")
        print("="*60 + "\n")
        
        self.is_running = True
        paused = False
        
        while self.is_running:
            if not paused:
                # Get 3D detections from mono+ToF vision
                detections_3d = self.get_3d_detections()
                
                # Filter for ready cotton with valid 3D position
                ready_cotton = [d for d in detections_3d 
                              if d['class'] == 'cotton_ready' and d['position_3d'] is not None]
                
                if len(ready_cotton) > 0:
                    print(f"\n✓ Found {len(ready_cotton)} ready cotton with depth")
                    
                    # Process each detection
                    for i, detection in enumerate(ready_cotton):
                        position_3d = detection['position_3d']
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
                    # Capture frame for visualization
                    frame = self.vision.capture_frame()
                    
                    # Annotate with detections
                    vis_frame = self.vision.visualize_detections(frame, detections_3d)
                    
                    # Add info overlay
                    cv2.putText(vis_frame, f"Picked: {self.picked_count}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(vis_frame, f"Ready: {len(ready_cotton)}", (10, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    cv2.imshow('Cotton Detection', cv2.cvtColor(vis_frame, cv2.COLOR_RGB2BGR))
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nStopping...")
                self.is_running = False
            elif key == ord('p'):
                paused = not paused
                status = "PAUSED" if paused else "RUNNING"
                print(f"\n{status}")
            elif key == ord('g'):
                print("\nTesting gripper...")
                home_pos = np.array([0.0, 0.0, 150.0])
                tendon = self.kinematics.inverse_kinematics(home_pos)
                servo = self.kinematics.tendon_to_servo_angles(tendon)
                print("  Opening gripper...")
                self.send_servo_command(servo, gripper_angle=0)
                time.sleep(1)
                print("  Closing gripper...")
                self.send_servo_command(servo, gripper_angle=180)
                time.sleep(1)
                print("  Opening gripper...")
                self.send_servo_command(servo, gripper_angle=0)
            elif key == ord('s'):
                frame = self.vision.capture_frame()
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"capture_{timestamp}.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                print(f"✓ Saved frame: capture_{timestamp}.jpg")
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Release resources"""
        print("\nCleaning up...")
        
        # Stop vision system
        self.vision.cleanup()
        
        # Close Arduino connection
        if self.arduino:
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
    ARDUINO_PORT = "COM3"  # Change to your Arduino port
    CAMERA_RESOLUTION = (640, 480)
    USE_MOCK_HARDWARE = False  # Set to True for testing without hardware
    
    # Check if files exist
    if not os.path.exists(MODEL_PATH):
        print(f"❌ Model not found: {MODEL_PATH}")
        print("Please ensure the YOLO model exists at model/best.pt")
        exit(1)
    
    # Initialize controller
    try:
        controller = PickingController(
            model_path=MODEL_PATH,
            arduino_port=ARDUINO_PORT,
            arduino_baud=115200,
            camera_resolution=CAMERA_RESOLUTION,
            use_mock_hardware=USE_MOCK_HARDWARE
        )
        
        # Run picking pipeline
        controller.run_picking_pipeline(show_visualization=True)
        
    except KeyboardInterrupt:
        print("\n\n❌ Interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nShutting down...")
