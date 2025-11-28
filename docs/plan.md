✅ 1. FULL PROJECT PLAN (High-Level)
Goal:

Autonomous cotton-picking robot using:

Dual stereo cameras (Arducam) for near 280° vision

YOLO model for ripe cotton detection

Continuum/tendon robot arm for picking

Arduino + PCA9685 to drive servos

Laptop for AI + depth fusion

PHASES
Phase 1 — Vision Setup (Dual Stereo Cameras)

Mount 2 Arducam stereo modules on a semicircular 3D-printed arc

Calibrate each camera

Perform stereo depth mapping

Create 270° stitched field-of-view

Test YOLO detection on full FOV

Phase 2 — Continuum Arm Assembly

Print OpenCR-Hardware TDCR parts (vertebrae, backbone guides, servo base)

Install 3 tendons + 3 high-torque servos

Test bending in 3 axes

Add gripper

Phase 3 — Hardware Integration

Connect laptop → cameras

Laptop → Arduino (Serial)

Arduino → PCA9685 → servos

Test servo movements mapped to 3D coordinates

Phase 4 — Picking Pipeline

Detect ripe cotton

Compute 3D point (X, Y, Z) from stereo

Move continuum arm tip to target

Gripper closes

Pick → drop

✅ 2. COMPONENT LIST (Fits inside ₹20,000)
Vision

✓ 2× Arducam Stereo USB Camera (OV9281, OV2311, or similar) – ₹4,500 each
Total: ₹9,000

Control

✓ Arduino Uno / Nano – ₹300
✓ PCA9685 16-channel servo driver – ₹300
✓ USB 3.0 cables – ₹200

Motion / Robot Arm

✓ 3× MG996R or DS3225 servos – ₹1,200 to ₹1,800
✓ Fishing line tendons (strong nylon) – ₹80
✓ 1× SG90 or MG90S for gripper – ₹150–₹180
✓ Power: 6V 10A DC supply – ₹800
✓ Wires, headers, bolts – ₹200
✓ 3D printing filament PLA (500g) – ₹350
✓ TPU (optional) – ₹300

Frame

✓ Base plate (wood/aluminium) – ₹150
✓ Cable ties, screws – ₹80 ~

TOTAL

≈ ₹14,500–₹16,000 (leaves ₹4,000 spare)

✅ 3. REQUIRED STL FILE PACKS
A. Continuum Arm (TDCR) – Official STLs

Source: OpenCR-Hardware by Continuum Robotics Lab
Link:
https://github.com/ContinuumRoboticsLab/OpenCR-Hardware

✅ 4. CONTINUUM ARM SERVO LAYOUT
Tendon-Driven Section

3 tendons at 120° spacing

Route them through spacer disks

Tie to 3 servo drums at base

             O Tendon A
          /     |     \
    Servo A   Backbone   Servo B
          \     |     /
             O Tendon C
                Servo C

Servos:

Servo A → Tendon A

Servo B → Tendon B

Servo C → Tendon C

Gripper:

1 servo mounted at tip or tendon-actuated.

✅ 5. WIRING DIAGRAM
         [Laptop]
             │ USB 3.0
  ┌──────────┴───────────┐
  │                      │
[Arducam L]        [Arducam R]
   │ USB 3.0           │ USB 3.0
   └──────────────┬────┘
                  │
            YOLO + Depth Fusion
                  │ Serial (USB)
                  ▼
              [Arduino]
                  │ I2C (SDA/SCL)
                  ▼
             [PCA9685 Board]
         ┌────────┼───────────┬──────────┐
         ▼        ▼           ▼          ▼
     Servo A  Servo B      Servo C    Servo D (Gripper)
         │        │           │          │
         └────────┴───────────┴──────────┘
                (5–6V 10A Power Supply)


All grounds must be common.

✅ 6. SOFTWARE PIPELINE
Laptop Side (Python)
1. Capture dual cameras:
left = cv2.VideoCapture(0)
right = cv2.VideoCapture(1)

2. Stereo depth:

Calibrate cameras

Use StereoBM or SGBM

Generate depth map

3. YOLO detection:

Run YOLOv8 or YOLOv11 on stitched image

For each cotton detection → find center (u, v)

4. Determine 3D coordinates:

Z = depth_map[v, u]

Back-project to camera coordinates

5. Send command to Arduino:
ser.write(f"S1:{a} S2:{b} S3:{c} S4:{g}\n".encode())

Arduino Side

Receive commands

Parse angles

Write to PCA9685

Move servos

✅ 7. ASSEMBLY INSTRUCTIONS
Step 1 — Print Parts

Print continuum disks and base

Print curved dual-camera arc

Print camera holders

Print gripper

Print servo base

Step 2 — Install Cameras

Attach both cameras on arc

Angle ~135° apart

Step 3 — Build Arm

Insert backbone

Insert spacer disks

Route tendons

Fix tendons to servo drums

Step 4 — Electronics

Mount servos

Connect PCA9685

Connect power

Connect Arduino

Step 5 — Software

Calibrate stereo cameras

Test depth

Train YOLO

Integrate detection

Send commands to Arduino

Test arm motion

Add gripper logic