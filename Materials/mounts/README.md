# 3D Printable Mounts for Arducam + Jetson Nano

### Dimensions
- **Jetson Nano mounting holes**: 58mm x 58mm (M3 screws)
- **Arducam PCB**: 25mm x 24mm
- **Camera lens height**: ~8mm from PCB
- **CSI cable**: Keep under 150mm for signal integrity

### Camera Position for Cotton Detection
```
Recommended Setup:
- Height above cotton: 20-40cm
- Viewing angle: 30-45° downward
- Field of view: Ensure 30-50cm coverage area
- Clearance: 10cm from gripper to avoid obstruction
```

### Print Settings
```
Layer Height: 0.2mm (0.15mm for fine details)
Infill: 20-30%
Material: PLA (indoor) or PETG (outdoor/heat)
Supports: Yes (for overhangs > 45°)
Adhesion: Brim or raft recommended
```


### How to Use OpenSCAD Design:
1. Download OpenSCAD: https://openscad.org/downloads.html
2. Copy the code above to a file: `arducam_mount.scad`
3. Open in OpenSCAD
4. Adjust parameters (camera_angle, dimensions)
5. Press F6 to render
6. Export as STL: File → Export → Export as STL


## Recommended Print Order

1. **Test print first**: Print camera mount only (small, fast)
2. **Check fit**: Verify Arducam fits before printing full assembly
3. **Print bracket**: Jetson mounting bracket or robot arm mount
4. **Final assembly**: Combine all parts

## Assembly Hardware Needed

- M2.5 screws (8mm) x4 - for camera to mount
- M3 screws (10mm) x4 - for mount to Jetson/robot
- M3 nuts x4
- Optional: Nylon washers for vibration dampening

## Testing After Installation

```bash
# On Jetson Nano, verify camera detection
ls /dev/video*

# Test camera view
nvgstcapture-1.0

# Run cotton detection
python3 jetson_arducam_detection.py
```

