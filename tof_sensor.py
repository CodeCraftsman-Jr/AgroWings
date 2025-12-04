"""
VL53L5CX Time-of-Flight Sensor Integration
Provides depth sensing for cotton picking robot using VL53L5CX 8x8 multi-zone ranging sensor
"""

import time
import numpy as np

try:
    import board
    import busio
    from adafruit_vl53l5cx import VL53L5CX
    VL53L5CX_AVAILABLE = True
except ImportError:
    print("Warning: VL53L5CX library not available. Install with: pip install adafruit-circuitpython-vl53l5cx")
    VL53L5CX_AVAILABLE = False


class ToFDepthSensor:
    """
    Wrapper class for VL53L5CX Time-of-Flight depth sensor
    
    Features:
    - 8x8 zone ranging (64 depth measurements)
    - Configurable ranging frequency (1-60 Hz)
    - Integration period adjustment
    - Pixel-to-zone mapping for camera fusion
    """
    
    def __init__(self, ranging_freq=15, resolution=8):
        """
        Initialize VL53L5CX sensor
        
        Args:
            ranging_freq (int): Measurement frequency in Hz (1-60)
            resolution (int): Resolution mode (4 or 8 for 4x4 or 8x8)
        """
        if not VL53L5CX_AVAILABLE:
            raise ImportError("VL53L5CX library not installed")
        
        self.resolution = resolution
        self.ranging_freq = ranging_freq
        
        # Initialize I2C and sensor
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.vl53 = VL53L5CX(i2c)
            
            # Configure sensor
            self.vl53.resolution = resolution  # 8x8 or 4x4
            self.vl53.ranging_frequency = ranging_freq  # Hz
            
            # Start ranging
            self.vl53.start_ranging()
            
            print(f"VL53L5CX initialized: {resolution}x{resolution} @ {ranging_freq}Hz")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize VL53L5CX: {e}")
        
        # Cached depth map
        self._depth_map = None
        self._last_update = 0
        
    def get_depth_map(self):
        """
        Get current 8x8 or 4x4 depth map
        
        Returns:
            numpy.ndarray: Depth map in millimeters (resolution x resolution)
        """
        if self.vl53.data_ready:
            # Read distance data
            distance_data = self.vl53.distance
            
            # Convert to numpy array and reshape
            self._depth_map = np.array(distance_data).reshape(
                (self.resolution, self.resolution)
            )
            self._last_update = time.time()
        
        return self._depth_map
    
    def get_depth_at_pixel(self, pixel_x, pixel_y, frame_width, frame_height):
        """
        Get depth at specific pixel coordinates by mapping to ToF zone
        
        Args:
            pixel_x (int): Pixel x-coordinate in camera frame
            pixel_y (int): Pixel y-coordinate in camera frame
            frame_width (int): Camera frame width in pixels
            frame_height (int): Camera frame height in pixels
            
        Returns:
            float: Depth in millimeters at that pixel location, or None if unavailable
        """
        # Get latest depth map
        depth_map = self.get_depth_map()
        
        if depth_map is None:
            return None
        
        # Map pixel coordinates to ToF zone indices
        zone_x = int((pixel_x / frame_width) * self.resolution)
        zone_y = int((pixel_y / frame_height) * self.resolution)
        
        # Clamp to valid range
        zone_x = max(0, min(zone_x, self.resolution - 1))
        zone_y = max(0, min(zone_y, self.resolution - 1))
        
        # Return depth from corresponding zone
        depth_mm = depth_map[zone_y, zone_x]
        
        # Filter invalid readings (VL53L5CX returns high values for errors)
        if depth_mm > 4000:  # Max valid range ~4m
            return None
        
        return depth_mm
    
    def get_average_depth_in_bbox(self, bbox, frame_width, frame_height):
        """
        Get average depth within a bounding box
        
        Args:
            bbox (tuple): Bounding box (x1, y1, x2, y2) in pixels
            frame_width (int): Camera frame width
            frame_height (int): Camera frame height
            
        Returns:
            float: Average depth in millimeters, or None if no valid readings
        """
        x1, y1, x2, y2 = bbox
        
        # Get center point
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        # For better accuracy, sample multiple points within bbox
        sample_points = [
            (center_x, center_y),  # Center
            (x1 + (x2-x1)*0.3, y1 + (y2-y1)*0.3),  # Top-left inner
            (x1 + (x2-x1)*0.7, y1 + (y2-y1)*0.3),  # Top-right inner
            (x1 + (x2-x1)*0.3, y1 + (y2-y1)*0.7),  # Bottom-left inner
            (x1 + (x2-x1)*0.7, y1 + (y2-y1)*0.7),  # Bottom-right inner
        ]
        
        depths = []
        for px, py in sample_points:
            depth = self.get_depth_at_pixel(
                int(px), int(py), frame_width, frame_height
            )
            if depth is not None and depth < 4000:
                depths.append(depth)
        
        if not depths:
            return None
        
        # Return median for robustness against outliers
        return float(np.median(depths))
    
    def get_depth_statistics(self):
        """
        Get statistics about current depth map
        
        Returns:
            dict: Statistics including min, max, mean, std depth
        """
        depth_map = self.get_depth_map()
        
        if depth_map is None:
            return None
        
        # Filter valid readings
        valid_depths = depth_map[depth_map < 4000]
        
        if len(valid_depths) == 0:
            return None
        
        return {
            'min_depth_mm': float(np.min(valid_depths)),
            'max_depth_mm': float(np.max(valid_depths)),
            'mean_depth_mm': float(np.mean(valid_depths)),
            'std_depth_mm': float(np.std(valid_depths)),
            'valid_zones': len(valid_depths),
            'total_zones': self.resolution * self.resolution
        }
    
    def visualize_depth_map(self):
        """
        Print ASCII visualization of depth map
        """
        depth_map = self.get_depth_map()
        
        if depth_map is None:
            print("No depth data available")
            return
        
        print("\nDepth Map (mm):")
        print("=" * (self.resolution * 6 + 1))
        
        for row in depth_map:
            row_str = ""
            for depth in row:
                if depth > 4000:
                    row_str += "  ----"
                else:
                    row_str += f"{depth:6.0f}"
            print(row_str)
        
        print("=" * (self.resolution * 6 + 1))
        
        stats = self.get_depth_statistics()
        if stats:
            print(f"Range: {stats['min_depth_mm']:.0f} - {stats['max_depth_mm']:.0f} mm")
            print(f"Mean: {stats['mean_depth_mm']:.0f} mm (±{stats['std_depth_mm']:.0f})")
            print(f"Valid zones: {stats['valid_zones']}/{stats['total_zones']}")
    
    def stop(self):
        """Stop ranging and cleanup"""
        try:
            self.vl53.stop_ranging()
            print("VL53L5CX stopped")
        except Exception as e:
            print(f"Error stopping sensor: {e}")
    
    def __del__(self):
        """Cleanup on deletion"""
        self.stop()


class MockToFDepthSensor:
    """
    Mock ToF sensor for testing without hardware
    Generates synthetic depth data
    """
    
    def __init__(self, ranging_freq=15, resolution=8):
        self.resolution = resolution
        self.ranging_freq = ranging_freq
        print(f"Mock VL53L5CX initialized: {resolution}x{resolution} @ {ranging_freq}Hz")
        
        # Generate random but consistent depth map
        np.random.seed(42)
        self._base_depth_map = 500 + np.random.randn(resolution, resolution) * 100
    
    def get_depth_map(self):
        """Return synthetic depth map"""
        # Add some noise for realism
        noise = np.random.randn(self.resolution, self.resolution) * 5
        return self._base_depth_map + noise
    
    def get_depth_at_pixel(self, pixel_x, pixel_y, frame_width, frame_height):
        """Get synthetic depth at pixel"""
        zone_x = int((pixel_x / frame_width) * self.resolution)
        zone_y = int((pixel_y / frame_height) * self.resolution)
        
        zone_x = max(0, min(zone_x, self.resolution - 1))
        zone_y = max(0, min(zone_y, self.resolution - 1))
        
        depth_map = self.get_depth_map()
        return float(depth_map[zone_y, zone_x])
    
    def get_average_depth_in_bbox(self, bbox, frame_width, frame_height):
        """Get synthetic average depth in bbox"""
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return self.get_depth_at_pixel(int(center_x), int(center_y), frame_width, frame_height)
    
    def get_depth_statistics(self):
        """Get statistics from synthetic depth map"""
        depth_map = self.get_depth_map()
        return {
            'min_depth_mm': float(np.min(depth_map)),
            'max_depth_mm': float(np.max(depth_map)),
            'mean_depth_mm': float(np.mean(depth_map)),
            'std_depth_mm': float(np.std(depth_map)),
            'valid_zones': self.resolution * self.resolution,
            'total_zones': self.resolution * self.resolution
        }
    
    def visualize_depth_map(self):
        """Print ASCII visualization"""
        depth_map = self.get_depth_map()
        print("\nMock Depth Map (mm):")
        print("=" * (self.resolution * 6 + 1))
        for row in depth_map:
            row_str = "".join([f"{depth:6.0f}" for depth in row])
            print(row_str)
        print("=" * (self.resolution * 6 + 1))
    
    def stop(self):
        """Mock stop"""
        pass


if __name__ == "__main__":
    """Test ToF sensor functionality"""
    print("Testing VL53L5CX ToF Sensor...")
    
    # Try real sensor, fall back to mock
    try:
        sensor = ToFDepthSensor(ranging_freq=15, resolution=8)
    except:
        print("\nReal sensor not available, using mock sensor")
        sensor = MockToFDepthSensor(ranging_freq=15, resolution=8)
    
    print("\nCollecting depth data...")
    time.sleep(0.5)  # Allow sensor to stabilize
    
    # Test depth map acquisition
    for i in range(5):
        sensor.visualize_depth_map()
        time.sleep(0.5)
    
    # Test pixel-to-zone mapping
    print("\n\nTesting pixel-to-zone mapping:")
    test_pixels = [
        (320, 240, 640, 480),  # Center of 640x480
        (100, 100, 640, 480),  # Top-left
        (540, 380, 640, 480),  # Bottom-right
    ]
    
    for px, py, fw, fh in test_pixels:
        depth = sensor.get_depth_at_pixel(px, py, fw, fh)
        print(f"Pixel ({px}, {py}) in {fw}x{fh} frame -> Depth: {depth:.1f} mm")
    
    # Test bbox averaging
    print("\n\nTesting bounding box depth averaging:")
    test_bbox = (200, 150, 440, 330)  # Example detection bbox
    avg_depth = sensor.get_average_depth_in_bbox(test_bbox, 640, 480)
    print(f"BBox {test_bbox} average depth: {avg_depth:.1f} mm")
    
    sensor.stop()
    print("\nTest complete!")
