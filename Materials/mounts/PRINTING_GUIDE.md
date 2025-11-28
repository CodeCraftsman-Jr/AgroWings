# 3D Printing Instructions

## Files Included

1. **arducam_mount.scad** - Full featured mount (customizable)
2. **simple_camera_mount.scad** - Quick minimal mount
3. **README.md** - Download links and specifications

## How to Generate STL Files

### Option 1: Use OpenSCAD (Recommended for Customization)

1. **Download OpenSCAD**: https://openscad.org/downloads.html
2. **Open file**: Open `arducam_mount.scad` or `simple_camera_mount.scad`
3. **Customize parameters** (in the file):
   ```openscad
   camera_tilt_angle = 40;      // Change viewing angle
   use_robot_clamp = true;      // true for robot arm, false for Jetson
   clamp_diameter = 25;         // Your robot arm diameter
   ```
4. **Render**: Press F6 (or Design → Render)
5. **Export STL**: File → Export → Export as STL
6. **Import to slicer**: Use Cura, PrusaSlicer, or your preferred slicer

### Option 2: Download Pre-made STL Files

Visit these links to download ready-to-print STL files:

- **Jetson + Camera Case**: https://www.thingiverse.com/thing:3518410
- **Adjustable Camera Mount**: https://www.thingiverse.com/thing:3938907
- **Robot Arm Mount**: https://www.thingiverse.com/thing:4891234

## Print Settings

### Recommended Settings
```
Printer: Any FDM printer (Ender 3, Prusa, etc.)
Material: PLA or PETG
Layer Height: 0.2mm (0.15mm for better quality)
Infill: 20-25%
Supports: YES (for overhangs)
Adhesion: Brim or Raft
Nozzle: 0.4mm
Print Speed: 50-60mm/s
```

### Material Choice
- **PLA**: Indoor use, easy to print
- **PETG**: Better for outdoor/hot environments
- **ABS**: Strongest but requires heated enclosure

## Slicing in Cura (Example)

1. Open Cura
2. Import STL file
3. Set profile:
   - Quality: Standard (0.2mm)
   - Infill: 20%
   - Support: Enable (touching buildplate)
   - Adhesion: Brim
4. Slice and save to SD card
5. Estimated print time: 2-4 hours (depending on design)

## Post-Processing

1. **Remove supports** carefully with pliers
2. **Clean holes** with 2mm and 3mm drill bits if needed
3. **Test fit** camera before final assembly
4. **Sand edges** (optional) with 220 grit sandpaper

## Hardware Required

### For Camera Mount:
- M2 x 8mm screws (x4) - Attach camera to mount
- M2 nuts (x4) - Optional for extra security

### For Robot Arm Attachment:
- M4 x 20mm bolts (x2) - Clamp to robot arm
- M4 nuts (x2)
- M4 washers (x4)

### For Jetson Nano Attachment:
- M3 x 10mm screws (x4) - Attach mount to Jetson
- M3 standoffs (x4) - Optional spacing

## Assembly Steps

1. **Print mount** with supports
2. **Remove supports** and clean
3. **Attach camera** to mount using M2 screws
4. **Thread CSI cable** through cable slot
5. **Attach mount** to robot arm or Jetson
6. **Connect CSI cable** to Jetson camera port
7. **Test camera** with `nvgstcapture-1.0`
8. **Adjust angle** if needed and tighten


## Testing After Print

```bash
# On Jetson Nano
ls /dev/video*          # Should show /dev/video0
nvgstcapture-1.0        # Test camera feed
python3 jetson_arducam_detection.py  # Run detection
```

