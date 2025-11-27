"""
High-Level Robot Control Interface
Provides simple API for controlling the cotton picking robot
Includes safety features, manual control, and testing utilities
"""

import cv2
import numpy as np
import time
import keyboard
from typing import Optional, Tuple
import threading

from picking_controller import PickingController
from continuum_kinematics import ContinuumKinematics

class RobotInterface:
    """
    High-level interface for robot control
    Provides safety, manual control, and utilities
    """
    
    def __init__(self, controller: PickingController):
        """
        Initialize robot interface
        
        Args:
            controller: PickingController instance
        """
        self.controller = controller
        self.kinematics = controller.kinematics
        
        # Safety parameters
        self.emergency_stop_active = False
        self.max_velocity = 50.0  # mm/s
        self.safety_margin = 10.0  # mm
        
        # Current state
        self.current_position = np.array([0.0, 0.0, 150.0])  # Home position
        self.current_gripper_state = True  # Open
        
        # Workspace boundaries (mm)
        self.workspace_bounds = {
            'x_min': -60.0, 'x_max': 60.0,
            'y_min': -60.0, 'y_max': 60.0,
            'z_min': 50.0, 'z_max': 150.0
        }
        
        print("Robot Interface initialized")
        print(f"Workspace: X[{self.workspace_bounds['x_min']:.0f},{self.workspace_bounds['x_max']:.0f}] "
              f"Y[{self.workspace_bounds['y_min']:.0f},{self.workspace_bounds['y_max']:.0f}] "
              f"Z[{self.workspace_bounds['z_min']:.0f},{self.workspace_bounds['z_max']:.0f}] mm")
    
    def emergency_stop(self):
        """Emergency stop - halt all motion and return to safe state"""
        print("\n🚨 EMERGENCY STOP ACTIVATED 🚨")
        self.emergency_stop_active = True
        
        # Return to home position
        self.return_home()
        
        print("Robot in safe state")
    
    def reset_emergency_stop(self):
        """Reset emergency stop"""
        print("Emergency stop reset")
        self.emergency_stop_active = False
    
    def check_safety(self, target_position: np.ndarray) -> Tuple[bool, str]:
        """
        Check if target position is safe
        
        Args:
            target_position: Target [x, y, z] in mm
        
        Returns:
            (is_safe, reason)
        """
        x, y, z = target_position
        
        # Check workspace bounds
        if x < self.workspace_bounds['x_min'] or x > self.workspace_bounds['x_max']:
            return False, f"X out of bounds: {x:.1f} mm"
        if y < self.workspace_bounds['y_min'] or y > self.workspace_bounds['y_max']:
            return False, f"Y out of bounds: {y:.1f} mm"
        if z < self.workspace_bounds['z_min'] or z > self.workspace_bounds['z_max']:
            return False, f"Z out of bounds: {z:.1f} mm"
        
        # Check kinematics workspace
        if not self.kinematics.is_in_workspace(target_position):
            return False, "Outside kinematic workspace"
        
        # Check emergency stop
        if self.emergency_stop_active:
            return False, "Emergency stop active"
        
        return True, "OK"
    
    def move_to_target(self, 
                      x: float, y: float, z: float,
                      check_safety: bool = True) -> bool:
        """
        Move to target position with safety checks
        
        Args:
            x, y, z: Target coordinates (mm)
            check_safety: Perform safety checks
        
        Returns:
            True if successful
        """
        target = np.array([x, y, z])
        
        # Safety check
        if check_safety:
            is_safe, reason = self.check_safety(target)
            if not is_safe:
                print(f"⚠ Move rejected: {reason}")
                return False
        
        # Execute move
        success = self.controller.move_to_position(target, self.current_gripper_state)
        
        if success:
            self.current_position = target
            print(f"✓ Moved to [{x:.1f}, {y:.1f}, {z:.1f}] mm")
        else:
            print(f"✗ Failed to move to [{x:.1f}, {y:.1f}, {z:.1f}] mm")
        
        return success
    
    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """
        Move relative to current position
        
        Args:
            dx, dy, dz: Relative movement (mm)
        
        Returns:
            True if successful
        """
        target = self.current_position + np.array([dx, dy, dz])
        return self.move_to_target(target[0], target[1], target[2])
    
    def return_home(self) -> bool:
        """
        Return to home position (straight up)
        
        Returns:
            True if successful
        """
        print("Returning home...")
        home = np.array([0.0, 0.0, 150.0])
        return self.move_to_target(home[0], home[1], home[2], check_safety=False)
    
    def set_gripper(self, state: bool) -> bool:
        """
        Set gripper state
        
        Args:
            state: True = open, False = closed
        
        Returns:
            True if successful
        """
        self.current_gripper_state = state
        success = self.controller.move_to_position(self.current_position, state)
        
        status = "opened" if state else "closed"
        if success:
            print(f"✓ Gripper {status}")
        else:
            print(f"✗ Failed to {status} gripper")
        
        return success
    
    def pick_at_position(self, x: float, y: float, z: float) -> bool:
        """
        Execute full picking sequence at position
        
        Args:
            x, y, z: Target coordinates (mm)
        
        Returns:
            True if successful
        """
        target = np.array([x, y, z])
        
        # Safety check
        is_safe, reason = self.check_safety(target)
        if not is_safe:
            print(f"⚠ Pick rejected: {reason}")
            return False
        
        # Execute pick
        return self.controller.pick_cotton(target)
    
    def manual_control(self):
        """
        Interactive manual control mode
        Uses keyboard for control
        """
        print("\n" + "="*60)
        print("MANUAL CONTROL MODE")
        print("="*60)
        print("\nKeyboard controls:")
        print("  Arrow keys - Move X/Y")
        print("  W/S - Move Z up/down")
        print("  G - Toggle gripper")
        print("  H - Return home")
        print("  ESC - Exit manual control")
        print("  SPACE - Emergency stop")
        print("="*60 + "\n")
        
        step_size = 5.0  # mm per keypress
        
        try:
            while True:
                if keyboard.is_pressed('esc'):
                    print("\nExiting manual control")
                    break
                
                if keyboard.is_pressed('space'):
                    self.emergency_stop()
                    time.sleep(0.5)
                
                # Movement controls
                elif keyboard.is_pressed('up'):
                    self.move_relative(0, step_size, 0)
                    time.sleep(0.2)
                elif keyboard.is_pressed('down'):
                    self.move_relative(0, -step_size, 0)
                    time.sleep(0.2)
                elif keyboard.is_pressed('left'):
                    self.move_relative(-step_size, 0, 0)
                    time.sleep(0.2)
                elif keyboard.is_pressed('right'):
                    self.move_relative(step_size, 0, 0)
                    time.sleep(0.2)
                elif keyboard.is_pressed('w'):
                    self.move_relative(0, 0, step_size)
                    time.sleep(0.2)
                elif keyboard.is_pressed('s'):
                    self.move_relative(0, 0, -step_size)
                    time.sleep(0.2)
                
                # Gripper control
                elif keyboard.is_pressed('g'):
                    self.set_gripper(not self.current_gripper_state)
                    time.sleep(0.3)
                
                # Home
                elif keyboard.is_pressed('h'):
                    self.return_home()
                    time.sleep(0.3)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nManual control interrupted")
    
    def test_workspace(self, num_points: int = 10):
        """
        Test workspace by moving to random positions
        
        Args:
            num_points: Number of positions to test
        """
        print("\n" + "="*60)
        print("WORKSPACE TEST")
        print("="*60)
        print(f"\nTesting {num_points} random positions...")
        
        # Return to home first
        self.return_home()
        time.sleep(1.0)
        
        success_count = 0
        
        for i in range(num_points):
            # Generate random position within bounds
            x = np.random.uniform(self.workspace_bounds['x_min'], 
                                 self.workspace_bounds['x_max'])
            y = np.random.uniform(self.workspace_bounds['y_min'], 
                                 self.workspace_bounds['y_max'])
            z = np.random.uniform(self.workspace_bounds['z_min'], 
                                 self.workspace_bounds['z_max'])
            
            print(f"\nTest {i+1}/{num_points}: [{x:.1f}, {y:.1f}, {z:.1f}] mm")
            
            if self.move_to_target(x, y, z):
                success_count += 1
                time.sleep(0.5)
            else:
                print("  Position unreachable")
        
        # Return home
        self.return_home()
        
        print("\n" + "="*60)
        print(f"WORKSPACE TEST COMPLETE")
        print(f"Success rate: {success_count}/{num_points} ({100*success_count/num_points:.1f}%)")
        print("="*60)
    
    def test_picking_sequence(self):
        """Test complete picking sequence at safe position"""
        print("\n" + "="*60)
        print("PICKING SEQUENCE TEST")
        print("="*60)
        
        # Test position (safe, reachable)
        test_position = np.array([20.0, 10.0, 120.0])
        
        print(f"\nTest position: [{test_position[0]:.1f}, {test_position[1]:.1f}, {test_position[2]:.1f}] mm")
        print("\nExecuting picking sequence...")
        
        success = self.pick_at_position(test_position[0], test_position[1], test_position[2])
        
        if success:
            print("\n✓ Picking sequence test PASSED")
        else:
            print("\n✗ Picking sequence test FAILED")
        
        print("="*60)
    
    def calibrate_home_position(self):
        """
        Interactive calibration of home position
        Adjust servos to find neutral/home configuration
        """
        print("\n" + "="*60)
        print("HOME POSITION CALIBRATION")
        print("="*60)
        print("\nManually adjust servos to find neutral position")
        print("The arm should be straight up with no bending")
        print("\nUse arrow keys to adjust, ENTER when done")
        print("="*60 + "\n")
        
        # Start with current home position
        servo_angles = np.array([90, 90, 90])  # Neutral
        adjustment = 1  # degrees per step
        
        try:
            while True:
                print(f"\rServo angles: S1:{servo_angles[0]:.0f}° S2:{servo_angles[1]:.0f}° S3:{servo_angles[2]:.0f}°", end='')
                
                # Send to Arduino
                self.controller.send_servo_command(servo_angles, 0)
                
                if keyboard.is_pressed('enter'):
                    break
                elif keyboard.is_pressed('1') and keyboard.is_pressed('up'):
                    servo_angles[0] = min(180, servo_angles[0] + adjustment)
                elif keyboard.is_pressed('1') and keyboard.is_pressed('down'):
                    servo_angles[0] = max(0, servo_angles[0] - adjustment)
                elif keyboard.is_pressed('2') and keyboard.is_pressed('up'):
                    servo_angles[1] = min(180, servo_angles[1] + adjustment)
                elif keyboard.is_pressed('2') and keyboard.is_pressed('down'):
                    servo_angles[1] = max(0, servo_angles[1] - adjustment)
                elif keyboard.is_pressed('3') and keyboard.is_pressed('up'):
                    servo_angles[2] = min(180, servo_angles[2] + adjustment)
                elif keyboard.is_pressed('3') and keyboard.is_pressed('down'):
                    servo_angles[2] = max(0, servo_angles[2] - adjustment)
                
                time.sleep(0.1)
            
            print(f"\n\n✓ Home position calibrated:")
            print(f"  Servo offsets: {servo_angles}")
            print("\nUpdate these values in continuum_kinematics.py")
            
        except KeyboardInterrupt:
            print("\nCalibration cancelled")


# Example usage and testing
if __name__ == "__main__":
    import os
    
    print("="*60)
    print("ROBOT INTERFACE - TESTING MODE")
    print("="*60)
    
    # Configuration
    MODEL_PATH = r"model\best.pt"
    CALIBRATION_FILE = r"calibration_data\stereo_calibration.json"
    ARDUINO_PORT = "COM3"
    
    if not os.path.exists(MODEL_PATH):
        print(f"❌ Model not found: {MODEL_PATH}")
        exit(1)
    
    if not os.path.exists(CALIBRATION_FILE):
        print(f"⚠ Calibration file not found")
        CALIBRATION_FILE = None
    
    # Initialize controller
    from picking_controller import PickingController
    
    controller = PickingController(
        model_path=MODEL_PATH,
        calibration_file=CALIBRATION_FILE,
        arduino_port=ARDUINO_PORT
    )
    
    # Create interface
    interface = RobotInterface(controller)
    
    # Menu
    while True:
        print("\n" + "="*60)
        print("ROBOT INTERFACE MENU")
        print("="*60)
        print("1. Test workspace coverage")
        print("2. Test picking sequence")
        print("3. Manual control mode")
        print("4. Calibrate home position")
        print("5. Move to home")
        print("6. Emergency stop test")
        print("7. Exit")
        print("="*60)
        
        choice = input("\nSelect option (1-7): ").strip()
        
        if choice == "1":
            interface.test_workspace(num_points=10)
        elif choice == "2":
            interface.test_picking_sequence()
        elif choice == "3":
            interface.manual_control()
        elif choice == "4":
            interface.calibrate_home_position()
        elif choice == "5":
            interface.return_home()
        elif choice == "6":
            interface.emergency_stop()
            interface.reset_emergency_stop()
        elif choice == "7":
            print("\nExiting...")
            interface.return_home()
            controller.cleanup()
            break
        else:
            print("Invalid option")
