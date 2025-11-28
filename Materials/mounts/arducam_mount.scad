// Arducam IMX219 Camera Mount for Cotton Picking Robot
// Designed for NVIDIA Jetson Nano
// Print Settings: 0.2mm layer, 20% infill, PLA or PETG

// ============== CUSTOMIZABLE PARAMETERS ==============

// Camera specifications
camera_pcb_width = 25;      // Arducam PCB width (mm)
camera_pcb_height = 24;     // Arducam PCB height (mm)
camera_pcb_thickness = 1.6; // PCB thickness (mm)
camera_hole_spacing = 21;   // Distance between mounting holes (mm)
camera_hole_diameter = 2.2; // M2 screw holes (mm)

// Mount plate dimensions
plate_width = 45;           // Base plate width (mm)
plate_length = 40;          // Base plate length (mm)
plate_thickness = 4;        // Base plate thickness (mm)

// Viewing angle
camera_tilt_angle = 40;     // Downward tilt angle (degrees)

// Jetson Nano mounting
jetson_hole_spacing = 58;   // Jetson Nano mounting hole spacing (mm)
jetson_hole_diameter = 3.2; // M3 screw holes (mm)

// Robot arm attachment
use_robot_clamp = true;     // Set to false for Jetson direct mount
clamp_diameter = 25;        // Robot arm tube diameter (mm)

// ============== MODULES ==============

module camera_holder() {
    difference() {
        // Main holder body
        union() {
            // Base plate
            cube([plate_width, plate_length, plate_thickness]);
            
            // Raised camera platform
            translate([5, 5, plate_thickness])
                cube([camera_pcb_width + 4, camera_pcb_height + 4, 2]);
        }
        
        // Camera mounting screw holes (M2)
        hole_offset_x = (plate_width - camera_hole_spacing) / 2;
        hole_offset_y = (plate_length - camera_hole_spacing) / 2;
        
        translate([hole_offset_x, hole_offset_y, -1])
            cylinder(h=plate_thickness+4, d=camera_hole_diameter, $fn=20);
        translate([hole_offset_x + camera_hole_spacing, hole_offset_y, -1])
            cylinder(h=plate_thickness+4, d=camera_hole_diameter, $fn=20);
        translate([hole_offset_x, hole_offset_y + camera_hole_spacing, -1])
            cylinder(h=plate_thickness+4, d=camera_hole_diameter, $fn=20);
        translate([hole_offset_x + camera_hole_spacing, hole_offset_y + camera_hole_spacing, -1])
            cylinder(h=plate_thickness+4, d=camera_hole_diameter, $fn=20);
        
        // Cable access slot
        translate([plate_width/2 - 6, -1, plate_thickness - 1])
            cube([12, 10, 3]);
    }
}

module angle_bracket() {
    rotate([camera_tilt_angle, 0, 0]) {
        // Vertical support
        cube([plate_width, plate_thickness, 35]);
        
        // Horizontal support
        translate([0, 0, 30])
            cube([plate_width, 15, plate_thickness]);
    }
}

module robot_clamp() {
    difference() {
        // Clamp body
        translate([0, -clamp_diameter/2 - 5, 0])
            cube([plate_width, clamp_diameter + 10, 25]);
        
        // Tube hole
        translate([plate_width/2, 0, 12])
            rotate([0, 90, 0])
                cylinder(h=plate_width+2, d=clamp_diameter + 0.5, center=true, $fn=50);
        
        // Tightening slot
        translate([plate_width/2 - 1.5, clamp_diameter/2 + 3, -1])
            cube([3, 8, 27]);
        
        // Bolt holes for clamping
        translate([plate_width/2 - 10, clamp_diameter/2 + 6, 12])
            rotate([90, 0, 0])
                cylinder(h=15, d=4, $fn=20);
        translate([plate_width/2 + 10, clamp_diameter/2 + 6, 12])
            rotate([90, 0, 0])
                cylinder(h=15, d=4, $fn=20);
    }
}

module jetson_mount_bracket() {
    difference() {
        // Base plate
        cube([70, 70, plate_thickness]);
        
        // Jetson Nano mounting holes (58mm spacing)
        hole_offset = (70 - jetson_hole_spacing) / 2;
        
        translate([hole_offset, hole_offset, -1])
            cylinder(h=plate_thickness+2, d=jetson_hole_diameter, $fn=20);
        translate([hole_offset + jetson_hole_spacing, hole_offset, -1])
            cylinder(h=plate_thickness+2, d=jetson_hole_diameter, $fn=20);
        translate([hole_offset, hole_offset + jetson_hole_spacing, -1])
            cylinder(h=plate_thickness+2, d=jetson_hole_diameter, $fn=20);
        translate([hole_offset + jetson_hole_spacing, hole_offset + jetson_hole_spacing, -1])
            cylinder(h=plate_thickness+2, d=jetson_hole_diameter, $fn=20);
    }
}

// ============== ASSEMBLY ==============

// Camera holder with angle
camera_holder();
translate([0, plate_length, 0])
    angle_bracket();

// Choose mounting method
if (use_robot_clamp) {
    // Robot arm clamp mount
    translate([0, 45, 35])
        rotate([camera_tilt_angle, 0, 0])
            robot_clamp();
} else {
    // Jetson Nano direct mount
    translate([-12.5, 45, 35])
        rotate([camera_tilt_angle, 0, 0])
            jetson_mount_bracket();
}

// ============== PRINT INSTRUCTIONS ==============
// 1. Export this as STL: File > Export > Export as STL
// 2. Print settings:
//    - Layer height: 0.2mm
//    - Infill: 20-25%
//    - Material: PLA (indoor) or PETG (outdoor)
//    - Supports: YES (for angle bracket overhang)
//    - Adhesion: Brim recommended
// 3. Hardware needed:
//    - M2 x 8mm screws (x4) for camera
//    - M3 x 10mm screws (x4) for Jetson/robot mount
//    - M4 x 20mm bolts (x2) for robot clamp (if used)
//    - Nuts as needed
