// Simple Camera Mount - Minimal Design
// Quick print for testing camera position

camera_angle = 45;  // Adjust angle (30-60 degrees)
tube_diameter = 25; // Robot arm diameter

difference() {
    union() {
        // Camera plate
        cube([35, 30, 3]);
        
        // Angle support
        translate([0, 0, 3])
            rotate([camera_angle, 0, 0])
                cube([35, 3, 25]);
        
        // Back support
        translate([0, 0, 0])
            rotate([camera_angle, 0, 0])
                translate([0, 0, 22])
                    cube([35, 20, 3]);
    }
    
    // Camera holes M2 (21mm spacing)
    translate([7, 5, -1]) cylinder(h=5, d=2.2, $fn=20);
    translate([28, 5, -1]) cylinder(h=5, d=2.2, $fn=20);
    translate([7, 26, -1]) cylinder(h=5, d=2.2, $fn=20);
    translate([28, 26, -1]) cylinder(h=5, d=2.2, $fn=20);
}

// Tube clamp
translate([0, -tube_diameter/2 - 8, 0])
    rotate([camera_angle, 0, 0])
        translate([0, 0, 22])
            difference() {
                translate([0, tube_diameter/2 + 3, 0])
                    cube([35, tube_diameter + 10, 20]);
                
                // Tube hole
                translate([17.5, tube_diameter/2 + 8, 10])
                    rotate([0, 90, 0])
                        cylinder(h=40, d=tube_diameter + 0.5, center=true, $fn=40);
                
                // Clamp gap
                translate([16, tube_diameter + 10, -1])
                    cube([3, 5, 22]);
                
                // Bolt holes
                translate([10, tube_diameter + 12, 10])
                    rotate([90, 0, 0])
                        cylinder(h=10, d=4, $fn=20);
                translate([25, tube_diameter + 12, 10])
                    rotate([90, 0, 0])
                        cylinder(h=10, d=4, $fn=20);
            }
