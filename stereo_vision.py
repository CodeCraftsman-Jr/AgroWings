"""
Stereo Vision System for Cotton Picking Robot
Supports multiple camera types with auto-detection
Handles calibration, depth mapping, and 3D reconstruction
"""

import cv2
import numpy as np
import json
import os
from pathlib import Path
from typing import Tuple, Optional, Dict, List
import glob

class StereoVisionSystem:
    """
    Stereo vision system with camera adaptation and 3D reconstruction
    """
    
    def __init__(self, calibration_file: Optional[str] = None):
        """
        Initialize stereo vision system
        
        Args:
            calibration_file: Path to calibration data (JSON or YAML)
        """
        self.left_camera = None
        self.right_camera = None
        self.camera_type = None
        
        # Calibration parameters
        self.mtx_left = None  # Left camera matrix
        self.dist_left = None  # Left distortion coefficients
        self.mtx_right = None  # Right camera matrix
        self.dist_right = None  # Right distortion coefficients
        self.R = None  # Rotation matrix
        self.T = None  # Translation vector
        self.E = None  # Essential matrix
        self.F = None  # Fundamental matrix
        
        # Rectification maps
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        
        # Stereo matcher
        self.stereo_matcher = None
        self.Q = None  # Disparity-to-depth mapping matrix
        
        # Camera properties
        self.baseline = None  # Distance between cameras (mm)
        self.focal_length = None  # Focal length (pixels)
        
        # Load calibration if provided
        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
    
    def detect_cameras(self, max_index: int = 10) -> Dict[str, List[int]]:
        """
        Auto-detect available cameras
        
        Args:
            max_index: Maximum camera index to check
            
        Returns:
            Dictionary with camera types and indices
        """
        print("Detecting cameras...")
        available_cameras = []
        camera_info = {}
        
        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fps = int(cap.get(cv2.CAP_PROP_FPS))
                    
                    # Try to identify camera type
                    camera_name = "Unknown"
                    if width == 1280 and height == 800:
                        camera_name = "Arducam OV9281 (Stereo)"
                    elif width == 1600 and height == 1300:
                        camera_name = "Arducam OV2311 (Stereo)"
                    elif width == 640 and height == 480:
                        camera_name = "Generic USB Webcam"
                    
                    available_cameras.append(i)
                    camera_info[i] = {
                        'name': camera_name,
                        'resolution': f"{width}x{height}",
                        'fps': fps
                    }
                    
                    print(f"✓ Camera {i}: {camera_name} ({width}x{height} @ {fps}fps)")
                cap.release()
        
        return {'available': available_cameras, 'info': camera_info}
    
    def setup_cameras(self, left_index: int = 0, right_index: int = 1, 
                     resolution: Tuple[int, int] = (1280, 720)) -> bool:
        """
        Setup and configure stereo camera pair
        
        Args:
            left_index: Index of left camera
            right_index: Index of right camera
            resolution: Desired resolution (width, height)
            
        Returns:
            True if successful
        """
        print(f"\nSetting up stereo cameras...")
        print(f"  Left camera: Index {left_index}")
        print(f"  Right camera: Index {right_index}")
        print(f"  Resolution: {resolution[0]}x{resolution[1]}")
        
        # Open left camera
        self.left_camera = cv2.VideoCapture(left_index)
        if not self.left_camera.isOpened():
            print(f"❌ Failed to open left camera (index {left_index})")
            return False
        
        # Open right camera
        self.right_camera = cv2.VideoCapture(right_index)
        if not self.right_camera.isOpened():
            print(f"❌ Failed to open right camera (index {right_index})")
            self.left_camera.release()
            return False
        
        # Set resolution
        self.left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.right_camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        
        # Verify resolution
        actual_width_l = int(self.left_camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height_l = int(self.left_camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_width_r = int(self.right_camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height_r = int(self.right_camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        print(f"✓ Left camera: {actual_width_l}x{actual_height_l}")
        print(f"✓ Right camera: {actual_width_r}x{actual_height_r}")
        
        if actual_width_l != actual_width_r or actual_height_l != actual_height_r:
            print("⚠ Warning: Camera resolutions don't match!")
        
        return True
    
    def capture_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Capture synchronized frames from both cameras
        
        Returns:
            Tuple of (left_frame, right_frame) or (None, None) if failed
        """
        if self.left_camera is None or self.right_camera is None:
            return None, None
        
        ret_l, frame_l = self.left_camera.read()
        ret_r, frame_r = self.right_camera.read()
        
        if ret_l and ret_r:
            return frame_l, frame_r
        return None, None
    
    def setup_stereo_matcher(self, method: str = 'SGBM'):
        """
        Setup stereo matching algorithm
        
        Args:
            method: 'BM' for StereoBM or 'SGBM' for StereoSGBM (recommended)
        """
        print(f"\nSetting up stereo matcher: {method}")
        
        if method == 'BM':
            # Basic block matching (faster but less accurate)
            self.stereo_matcher = cv2.StereoBM_create(numDisparities=16*10, blockSize=15)
        else:
            # Semi-Global Block Matching (slower but more accurate)
            window_size = 5
            min_disp = 0
            num_disp = 16*10  # Must be divisible by 16
            
            self.stereo_matcher = cv2.StereoSGBM_create(
                minDisparity=min_disp,
                numDisparities=num_disp,
                blockSize=window_size,
                P1=8 * 3 * window_size ** 2,
                P2=32 * 3 * window_size ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
        
        print("✓ Stereo matcher configured")
    
    def compute_depth_map(self, left_frame: np.ndarray, right_frame: np.ndarray,
                          rectify: bool = True) -> Optional[np.ndarray]:
        """
        Compute disparity/depth map from stereo pair
        
        Args:
            left_frame: Left camera frame
            right_frame: Right camera frame
            rectify: Apply rectification if calibration available
            
        Returns:
            Depth map (distance in mm) or None if failed
        """
        if self.stereo_matcher is None:
            self.setup_stereo_matcher()
        
        # Convert to grayscale
        gray_l = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
        
        # Apply rectification if calibration available
        if rectify and self.left_map1 is not None:
            gray_l = cv2.remap(gray_l, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            gray_r = cv2.remap(gray_r, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
        
        # Compute disparity
        disparity = self.stereo_matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
        
        # Convert disparity to depth
        if self.Q is not None:
            # Use Q matrix from calibration
            depth_map = cv2.reprojectImageTo3D(disparity, self.Q)
            # Extract Z coordinate (depth)
            depth = depth_map[:, :, 2]
        elif self.baseline is not None and self.focal_length is not None:
            # Use baseline and focal length
            # Depth = (baseline * focal_length) / disparity
            depth = np.zeros_like(disparity)
            mask = disparity > 0
            depth[mask] = (self.baseline * self.focal_length) / disparity[mask]
        else:
            # Return raw disparity
            depth = disparity
        
        return depth
    
    def get_3d_point(self, u: int, v: int, depth_map: np.ndarray) -> Optional[np.ndarray]:
        """
        Get 3D coordinates of a point from depth map
        
        Args:
            u: Pixel x-coordinate
            v: Pixel y-coordinate
            depth_map: Depth map from compute_depth_map()
            
        Returns:
            3D point [X, Y, Z] in camera coordinates (mm) or None
        """
        if depth_map is None or u < 0 or v < 0:
            return None
        
        if v >= depth_map.shape[0] or u >= depth_map.shape[1]:
            return None
        
        Z = depth_map[v, u]
        
        if Z <= 0 or np.isinf(Z) or np.isnan(Z):
            return None
        
        # Back-project to 3D
        if self.mtx_left is not None:
            fx = self.mtx_left[0, 0]
            fy = self.mtx_left[1, 1]
            cx = self.mtx_left[0, 2]
            cy = self.mtx_left[1, 2]
            
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
        else:
            # Assume simple pinhole model
            X = (u - depth_map.shape[1] / 2) * Z / 1000.0
            Y = (v - depth_map.shape[0] / 2) * Z / 1000.0
        
        return np.array([X, Y, Z])
    
    def load_calibration(self, filepath: str) -> bool:
        """
        Load calibration data from file
        
        Args:
            filepath: Path to calibration JSON file
            
        Returns:
            True if successful
        """
        print(f"\nLoading calibration from: {filepath}")
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Load matrices
            self.mtx_left = np.array(data['mtx_left'])
            self.dist_left = np.array(data['dist_left'])
            self.mtx_right = np.array(data['mtx_right'])
            self.dist_right = np.array(data['dist_right'])
            self.R = np.array(data['R'])
            self.T = np.array(data['T'])
            
            if 'E' in data:
                self.E = np.array(data['E'])
            if 'F' in data:
                self.F = np.array(data['F'])
            
            # Load camera properties
            self.baseline = data.get('baseline', None)
            self.focal_length = data.get('focal_length', None)
            
            # Compute rectification maps
            img_size = tuple(data['image_size'])
            R1, R2, P1, P2, self.Q, roi1, roi2 = cv2.stereoRectify(
                self.mtx_left, self.dist_left,
                self.mtx_right, self.dist_right,
                img_size, self.R, self.T,
                alpha=0
            )
            
            self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
                self.mtx_left, self.dist_left, R1, P1, img_size, cv2.CV_32FC1
            )
            self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
                self.mtx_right, self.dist_right, R2, P2, img_size, cv2.CV_32FC1
            )
            
            print("✓ Calibration loaded successfully")
            print(f"  Baseline: {self.baseline:.2f} mm")
            print(f"  Focal length: {self.focal_length:.2f} pixels")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to load calibration: {e}")
            return False
    
    def save_calibration(self, filepath: str, image_size: Tuple[int, int]) -> bool:
        """
        Save calibration data to file
        
        Args:
            filepath: Path to save calibration JSON
            image_size: Image dimensions (width, height)
            
        Returns:
            True if successful
        """
        try:
            data = {
                'mtx_left': self.mtx_left.tolist(),
                'dist_left': self.dist_left.tolist(),
                'mtx_right': self.mtx_right.tolist(),
                'dist_right': self.dist_right.tolist(),
                'R': self.R.tolist(),
                'T': self.T.tolist(),
                'image_size': list(image_size),
                'baseline': float(self.baseline) if self.baseline else None,
                'focal_length': float(self.focal_length) if self.focal_length else None
            }
            
            if self.E is not None:
                data['E'] = self.E.tolist()
            if self.F is not None:
                data['F'] = self.F.tolist()
            
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"✓ Calibration saved to: {filepath}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to save calibration: {e}")
            return False
    
    def release(self):
        """Release camera resources"""
        if self.left_camera is not None:
            self.left_camera.release()
        if self.right_camera is not None:
            self.right_camera.release()
        cv2.destroyAllWindows()
        print("✓ Cameras released")


def visualize_depth_map(depth_map: np.ndarray, title: str = "Depth Map") -> np.ndarray:
    """
    Create colorized visualization of depth map
    
    Args:
        depth_map: Depth map array
        title: Window title
        
    Returns:
        Colorized depth map
    """
    # Normalize and colorize
    depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_normalized = np.uint8(depth_normalized)
    depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    
    return depth_colormap


# Example usage
if __name__ == "__main__":
    print("="*60)
    print("Stereo Vision System Test")
    print("="*60)
    
    # Initialize system
    stereo = StereoVisionSystem()
    
    # Detect cameras
    cameras = stereo.detect_cameras()
    
    if len(cameras['available']) < 2:
        print("\n❌ Error: Need at least 2 cameras for stereo vision")
        print("Please connect two cameras and try again")
    else:
        print(f"\n✓ Found {len(cameras['available'])} cameras")
        
        # Setup cameras (use first two available)
        left_idx = cameras['available'][0]
        right_idx = cameras['available'][1]
        
        if stereo.setup_cameras(left_idx, right_idx):
            print("\n✓ Stereo cameras ready")
            print("\nPress 'q' to quit, 's' to save frame, 'd' to show depth map")
            
            # Setup stereo matcher
            stereo.setup_stereo_matcher('SGBM')
            
            while True:
                # Capture frames
                left_frame, right_frame = stereo.capture_frames()
                
                if left_frame is not None and right_frame is not None:
                    # Show frames
                    combined = np.hstack((left_frame, right_frame))
                    cv2.imshow('Stereo Cameras (Left | Right)', combined)
                    
                    key = cv2.waitKey(1) & 0xFF
                    
                    if key == ord('q'):
                        break
                    elif key == ord('d'):
                        # Compute and show depth map
                        print("Computing depth map...")
                        depth = stereo.compute_depth_map(left_frame, right_frame, rectify=False)
                        if depth is not None:
                            depth_viz = visualize_depth_map(depth)
                            cv2.imshow('Depth Map', depth_viz)
                    elif key == ord('s'):
                        # Save frames
                        timestamp = cv2.getTickCount()
                        cv2.imwrite(f'left_{timestamp}.jpg', left_frame)
                        cv2.imwrite(f'right_{timestamp}.jpg', right_frame)
                        print(f"✓ Frames saved")
            
            stereo.release()
        else:
            print("\n❌ Failed to setup cameras")
