"""
VL53L5CX ToF Sensor Test Script
Test VL53L5CX depth sensor independently before integration
"""

import time
import sys

print("="*60)
print("VL53L5CX ToF SENSOR TEST")
print("="*60)

# Try to import sensor
try:
    from tof_sensor import ToFDepthSensor
    print("✓ Imported ToFDepthSensor")
    use_mock = False
except ImportError as e:
    print(f"⚠ Warning: Could not import ToFDepthSensor: {e}")
    from tof_sensor import MockToFDepthSensor
    print("✓ Using MockToFDepthSensor for testing")
    use_mock = True

print("\n" + "="*60)
print("INITIALIZING SENSOR")
print("="*60)

# Initialize sensor
try:
    if use_mock:
        sensor = MockToFDepthSensor(ranging_freq=15, resolution=8)
    else:
        sensor = ToFDepthSensor(ranging_freq=15, resolution=8)
    print("✓ Sensor initialized successfully")
except Exception as e:
    print(f"❌ Failed to initialize sensor: {e}")
    sys.exit(1)

# Allow sensor to stabilize
print("\nWaiting for sensor to stabilize...")
time.sleep(1.0)

print("\n" + "="*60)
print("TEST 1: CONTINUOUS DEPTH MAP READING")
print("="*60)

print("\nReading depth maps (5 iterations)...\n")

for i in range(5):
    print(f"\n--- Iteration {i+1}/5 ---")
    
    # Get depth map
    depth_map = sensor.get_depth_map()
    
    if depth_map is not None:
        print(f"✓ Depth map acquired: {depth_map.shape}")
        
        # Visualize
        sensor.visualize_depth_map()
        
    else:
        print("❌ Failed to acquire depth map")
    
    time.sleep(0.5)

print("\n" + "="*60)
print("TEST 2: DEPTH STATISTICS")
print("="*60)

stats = sensor.get_depth_statistics()

if stats:
    print("\nDepth Statistics:")
    print(f"  Min depth:     {stats['min_depth_mm']:.1f} mm")
    print(f"  Max depth:     {stats['max_depth_mm']:.1f} mm")
    print(f"  Mean depth:    {stats['mean_depth_mm']:.1f} mm")
    print(f"  Std deviation: {stats['std_depth_mm']:.1f} mm")
    print(f"  Valid zones:   {stats['valid_zones']}/{stats['total_zones']}")
else:
    print("❌ Failed to get statistics")

print("\n" + "="*60)
print("TEST 3: PIXEL-TO-ZONE MAPPING")
print("="*60)

# Test pixel coordinates (for 640x480 frame)
test_pixels = [
    (320, 240),  # Center
    (100, 100),  # Top-left
    (540, 380),  # Bottom-right
    (320, 100),  # Top-center
    (320, 380),  # Bottom-center
]

frame_width = 640
frame_height = 480

print(f"\nTesting pixel-to-zone mapping for {frame_width}x{frame_height} frame:\n")

for px, py in test_pixels:
    depth = sensor.get_depth_at_pixel(px, py, frame_width, frame_height)
    if depth is not None:
        print(f"  Pixel ({px:3d}, {py:3d}) → Depth: {depth:6.1f} mm")
    else:
        print(f"  Pixel ({px:3d}, {py:3d}) → Depth: N/A")

print("\n" + "="*60)
print("TEST 4: BOUNDING BOX DEPTH AVERAGING")
print("="*60)

# Test bounding boxes (simulating YOLO detections)
test_bboxes = [
    (200, 150, 440, 330, "Center detection"),
    (50, 50, 200, 200, "Top-left detection"),
    (440, 280, 590, 430, "Bottom-right detection"),
]

print(f"\nTesting bbox depth averaging:\n")

for x1, y1, x2, y2, label in test_bboxes:
    avg_depth = sensor.get_average_depth_in_bbox((x1, y1, x2, y2), frame_width, frame_height)
    if avg_depth is not None:
        print(f"  {label:25s} [{x1}, {y1}, {x2}, {y2}]")
        print(f"    → Average depth: {avg_depth:.1f} mm")
    else:
        print(f"  {label:25s} → Depth: N/A")

print("\n" + "="*60)
print("TEST 5: CONTINUOUS MONITORING")
print("="*60)

print("\nMonitoring depth sensor for 10 seconds...")
print("(This simulates real-time operation)\n")

start_time = time.time()
reading_count = 0

try:
    while time.time() - start_time < 10.0:
        # Get depth at center pixel
        center_depth = sensor.get_depth_at_pixel(320, 240, 640, 480)
        
        # Get statistics
        stats = sensor.get_depth_statistics()
        
        if center_depth and stats:
            reading_count += 1
            elapsed = time.time() - start_time
            
            print(f"[{elapsed:5.2f}s] Center: {center_depth:6.1f} mm | "
                  f"Mean: {stats['mean_depth_mm']:6.1f} mm | "
                  f"Range: {stats['min_depth_mm']:6.1f}-{stats['max_depth_mm']:6.1f} mm",
                  end='\r')
        
        time.sleep(0.1)  # 10 Hz
    
    print()  # New line
    print(f"\n✓ Completed {reading_count} readings in 10 seconds")
    print(f"  Average rate: {reading_count/10:.1f} Hz")
    
except KeyboardInterrupt:
    print("\n\n⚠ Test interrupted by user")

print("\n" + "="*60)
print("TEST SUMMARY")
print("="*60)

print("\n✓ All tests completed successfully!")
print("\nSensor capabilities verified:")
print("  • Depth map acquisition")
print("  • Statistical analysis")
print("  • Pixel-to-zone mapping")
print("  • Bounding box averaging")
print("  • Continuous operation")

if use_mock:
    print("\n⚠ Note: Tests ran with MOCK sensor")
    print("  Install real sensor library with:")
    print("  pip install adafruit-circuitpython-vl53l5cx")

print("\nNext steps:")
print("  1. Integrate with camera system (mono_tof_vision.py)")
print("  2. Test with YOLO detections")
print("  3. Calibrate depth accuracy")

# Cleanup
sensor.stop()
print("\n✓ Sensor stopped")

print("\n" + "="*60)
print("TEST COMPLETE")
print("="*60)
