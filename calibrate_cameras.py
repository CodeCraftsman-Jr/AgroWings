"""
Camera Calibration Utility for Stereo Vision System
Interactive tool to calibrate stereo camera pairs
Saves calibration data for stereo_vision.py
"""

import cv2
import numpy as np
import os
import json
from pathlib import Path
from datetime import datetime
import glob

class CameraCalibration:
    """
    Interactive camera calibration for stereo systems
    """
    
    def __init__(self, 
                 checkerboard_size: tuple = (9, 6),
                 square_size: float = 25.0,  # mm
                 output_dir: str = "calibration_data"):
        """
        Initialize calibration tool
        
        Args:
            checkerboard_size: Inner corners (width, height)
            square_size: Size of checkerboard squares (mm)
            output_dir: Directory to save calibration data
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        self.output_dir = output_dir
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Calibration data storage
        self.left_images = []
        self.right_images = []
        self.left_corners = []
        self.right_corners = []
        
        # Termination criteria for corner refinement
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Prepare object points (3D points in real world space)
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 
                                     0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        print(f"Camera Calibration Tool")
        print(f"Checkerboard: {checkerboard_size[0]}x{checkerboard_size[1]} inner corners")
        print(f"Square size: {square_size} mm")
        print(f"Output directory: {output_dir}")
    
    def capture_calibration_images(self, 
                                   left_camera_idx: int = 0,
                                   right_camera_idx: int = 1,
                                   num_images: int = 20):
        """
        Interactive capture of calibration images
        
        Args:
            left_camera_idx: Left camera index
            right_camera_idx: Right camera index
            num_images: Number of image pairs to capture
        """
        print("\n" + "="*60)
        print("CALIBRATION IMAGE CAPTURE")
        print("="*60)
        print(f"\nTarget: {num_images} image pairs")
        print("\nInstructions:")
        print("  1. Print checkerboard pattern (see calibration_data/checkerboard.pdf)")
        print("  2. Place checkerboard in view of both cameras")
        print("  3. Press SPACE to capture when corners are detected")
        print("  4. Move checkerboard to different positions/angles")
        print("  5. Press 'q' to finish early")
        print("\nTips:")
        print("  - Capture at various distances (near, far)")
        print("  - Capture at various angles (tilted, rotated)")
        print("  - Cover all areas of camera view")
        print("  - Ensure checkerboard is flat and well-lit")
        print("="*60 + "\n")
        
        # Open cameras
        cap_left = cv2.VideoCapture(left_camera_idx)
        cap_right = cv2.VideoCapture(right_camera_idx)
        
        if not cap_left.isOpened() or not cap_right.isOpened():
            print("❌ Error: Could not open cameras")
            return False
        
        # Set resolution
        cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        captured = 0
        
        while captured < num_images:
            ret_l, frame_l = cap_left.read()
            ret_r, frame_r = cap_right.read()
            
            if not ret_l or not ret_r:
                print("❌ Failed to capture frames")
                break
            
            # Convert to grayscale
            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
            
            # Find checkerboard corners
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, self.checkerboard_size, None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, self.checkerboard_size, None)
            
            # Display frames
            display_l = frame_l.copy()
            display_r = frame_r.copy()
            
            if ret_l and ret_r:
                # Refine corners
                corners_l_refined = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), self.criteria)
                corners_r_refined = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), self.criteria)
                
                # Draw corners
                cv2.drawChessboardCorners(display_l, self.checkerboard_size, corners_l_refined, ret_l)
                cv2.drawChessboardCorners(display_r, self.checkerboard_size, corners_r_refined, ret_r)
                
                # Status text
                cv2.putText(display_l, "READY - Press SPACE", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_r, "READY - Press SPACE", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(display_l, "Checkerboard not found", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(display_r, "Checkerboard not found", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Progress indicator
            progress_text = f"Captured: {captured}/{num_images}"
            cv2.putText(display_l, progress_text, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.putText(display_r, progress_text, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            # Show frames
            combined = np.hstack((display_l, display_r))
            cv2.imshow('Calibration - Left | Right', combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and ret_l and ret_r:
                # Capture image pair
                self.left_images.append(gray_l)
                self.right_images.append(gray_r)
                self.left_corners.append(corners_l_refined)
                self.right_corners.append(corners_r_refined)
                
                # Save images
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(os.path.join(self.output_dir, f"left_{captured}_{timestamp}.jpg"), frame_l)
                cv2.imwrite(os.path.join(self.output_dir, f"right_{captured}_{timestamp}.jpg"), frame_r)
                
                captured += 1
                print(f"✓ Captured image pair {captured}/{num_images}")
                
            elif key == ord('q'):
                print(f"\nCapture stopped early ({captured} images)")
                break
        
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()
        
        if captured >= 10:
            print(f"\n✓ Captured {captured} image pairs (minimum 10 required)")
            return True
        else:
            print(f"\n❌ Insufficient images ({captured} < 10)")
            return False
    
    def calibrate_stereo(self) -> dict:
        """
        Perform stereo calibration using captured images
        
        Returns:
            Dictionary with calibration parameters
        """
        print("\n" + "="*60)
        print("STEREO CALIBRATION")
        print("="*60)
        
        if len(self.left_images) < 10:
            print("❌ Not enough images for calibration (minimum 10)")
            return None
        
        print(f"\nCalibrating with {len(self.left_images)} image pairs...")
        
        # Get image size
        img_size = self.left_images[0].shape[::-1]
        
        # Prepare object points
        objpoints = [self.objp for _ in range(len(self.left_images))]
        
        # Calibrate left camera
        print("  Calibrating left camera...")
        ret_l, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(
            objpoints, self.left_corners, img_size, None, None
        )
        print(f"    RMS error: {ret_l:.4f}")
        
        # Calibrate right camera
        print("  Calibrating right camera...")
        ret_r, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(
            objpoints, self.right_corners, img_size, None, None
        )
        print(f"    RMS error: {ret_r:.4f}")
        
        # Stereo calibration
        print("  Calibrating stereo pair...")
        flags = cv2.CALIB_FIX_INTRINSIC
        
        ret_stereo, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            objpoints,
            self.left_corners,
            self.right_corners,
            mtx_l, dist_l,
            mtx_r, dist_r,
            img_size,
            criteria=self.criteria,
            flags=flags
        )
        
        print(f"    Stereo RMS error: {ret_stereo:.4f}")
        
        # Calculate baseline and focal length
        baseline = np.linalg.norm(T)  # Distance between cameras
        focal_length = (mtx_l[0, 0] + mtx_r[0, 0]) / 2.0  # Average focal length
        
        print(f"\n  Baseline: {baseline:.2f} mm")
        print(f"  Focal length: {focal_length:.2f} pixels")
        print(f"  Left camera matrix:\n{mtx_l}")
        print(f"  Right camera matrix:\n{mtx_r}")
        
        # Package calibration data
        calibration_data = {
            'mtx_left': mtx_l,
            'dist_left': dist_l,
            'mtx_right': mtx_r,
            'dist_right': dist_r,
            'R': R,
            'T': T,
            'E': E,
            'F': F,
            'baseline': baseline,
            'focal_length': focal_length,
            'image_size': img_size,
            'rms_error_left': ret_l,
            'rms_error_right': ret_r,
            'rms_error_stereo': ret_stereo,
            'num_images': len(self.left_images),
            'checkerboard_size': self.checkerboard_size,
            'square_size': self.square_size
        }
        
        print("\n✓ Calibration complete!")
        
        return calibration_data
    
    def save_calibration(self, calibration_data: dict, filename: str = None):
        """
        Save calibration data to JSON file
        
        Args:
            calibration_data: Calibration dictionary
            filename: Output filename (auto-generated if None)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"stereo_calibration_{timestamp}.json"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # Convert numpy arrays to lists for JSON serialization
        json_data = {}
        for key, value in calibration_data.items():
            if isinstance(value, np.ndarray):
                json_data[key] = value.tolist()
            else:
                json_data[key] = value
        
        with open(filepath, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        print(f"\n✓ Calibration saved to: {filepath}")
        return filepath
    
    def load_from_images(self, left_pattern: str, right_pattern: str):
        """
        Load calibration images from disk
        
        Args:
            left_pattern: Glob pattern for left images
            right_pattern: Glob pattern for right images
        """
        left_files = sorted(glob.glob(left_pattern))
        right_files = sorted(glob.glob(right_pattern))
        
        if len(left_files) != len(right_files):
            print("❌ Mismatch in number of left and right images")
            return False
        
        print(f"Loading {len(left_files)} image pairs...")
        
        for left_file, right_file in zip(left_files, right_files):
            img_l = cv2.imread(left_file, cv2.IMREAD_GRAYSCALE)
            img_r = cv2.imread(right_file, cv2.IMREAD_GRAYSCALE)
            
            # Find corners
            ret_l, corners_l = cv2.findChessboardCorners(img_l, self.checkerboard_size, None)
            ret_r, corners_r = cv2.findChessboardCorners(img_r, self.checkerboard_size, None)
            
            if ret_l and ret_r:
                # Refine corners
                corners_l = cv2.cornerSubPix(img_l, corners_l, (11, 11), (-1, -1), self.criteria)
                corners_r = cv2.cornerSubPix(img_r, corners_r, (11, 11), (-1, -1), self.criteria)
                
                self.left_images.append(img_l)
                self.right_images.append(img_r)
                self.left_corners.append(corners_l)
                self.right_corners.append(corners_r)
                
                print(f"  ✓ {os.path.basename(left_file)}")
            else:
                print(f"  ✗ {os.path.basename(left_file)} - corners not found")
        
        print(f"\n✓ Loaded {len(self.left_images)} valid image pairs")
        return len(self.left_images) >= 10
    
    def generate_checkerboard_pdf(self):
        """Generate checkerboard pattern PDF for printing"""
        try:
            import matplotlib.pyplot as plt
            from matplotlib.backends.backend_pdf import PdfPages
            
            # Create checkerboard pattern
            board_width = self.checkerboard_size[0] + 1
            board_height = self.checkerboard_size[1] + 1
            
            checkerboard = np.zeros((board_height, board_width))
            checkerboard[::2, ::2] = 1
            checkerboard[1::2, 1::2] = 1
            
            # Save to PDF
            pdf_path = os.path.join(self.output_dir, "checkerboard_pattern.pdf")
            
            with PdfPages(pdf_path) as pdf:
                fig, ax = plt.subplots(figsize=(11, 8.5))  # Letter size
                ax.imshow(checkerboard, cmap='gray', interpolation='nearest')
                ax.set_title(f'Checkerboard Pattern - {self.checkerboard_size[0]}x{self.checkerboard_size[1]} inner corners\n'
                           f'Square size: {self.square_size} mm', fontsize=14)
                ax.axis('off')
                plt.tight_layout()
                pdf.savefig(fig, dpi=300)
                plt.close()
            
            print(f"\n✓ Checkerboard pattern saved to: {pdf_path}")
            print(f"  Print this PDF at actual size (no scaling)")
            print(f"  Mount on flat, rigid surface")
            
            return pdf_path
        except ImportError:
            print("⚠ matplotlib not installed - cannot generate PDF")
            print("  Download checkerboard pattern from:")
            print("  https://github.com/opencv/opencv/blob/master/doc/pattern.png")
            return None


# Main calibration workflow
if __name__ == "__main__":
    print("="*60)
    print("STEREO CAMERA CALIBRATION TOOL")
    print("="*60)
    
    # Initialize calibration
    calibrator = CameraCalibration(
        checkerboard_size=(9, 6),  # 9x6 inner corners
        square_size=25.0,          # 25mm squares
        output_dir="calibration_data"
    )
    
    # Generate checkerboard pattern
    print("\nGenerating checkerboard pattern...")
    calibrator.generate_checkerboard_pdf()
    
    print("\n" + "="*60)
    print("CALIBRATION OPTIONS")
    print("="*60)
    print("1. Capture new calibration images")
    print("2. Load existing images from disk")
    print("3. Exit")
    
    choice = input("\nSelect option (1-3): ").strip()
    
    if choice == "1":
        # Capture new images
        if calibrator.capture_calibration_images(
            left_camera_idx=0,
            right_camera_idx=1,
            num_images=20
        ):
            # Perform calibration
            calib_data = calibrator.calibrate_stereo()
            
            if calib_data is not None:
                # Save calibration
                calibrator.save_calibration(calib_data)
                
                print("\n" + "="*60)
                print("CALIBRATION COMPLETE!")
                print("="*60)
                print("\nNext steps:")
                print("1. Use the saved calibration file with stereo_vision.py")
                print("2. Test depth mapping with the calibrated cameras")
                print("3. Integrate with picking pipeline")
    
    elif choice == "2":
        # Load from disk
        print("\nEnter image patterns (use wildcards):")
        left_pattern = input("  Left images: ").strip()
        right_pattern = input("  Right images: ").strip()
        
        if calibrator.load_from_images(left_pattern, right_pattern):
            calib_data = calibrator.calibrate_stereo()
            
            if calib_data is not None:
                calibrator.save_calibration(calib_data)
    
    else:
        print("\nExiting...")
