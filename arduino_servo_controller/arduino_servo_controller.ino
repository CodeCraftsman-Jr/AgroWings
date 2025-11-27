/*
 * Arduino Servo Controller for Cotton Picking Robot
 * Controls 4 servos via PCA9685 PWM driver
 * Receives commands via Serial from laptop
 * 
 * Hardware:
 * - Arduino Uno/Nano
 * - PCA9685 16-Channel PWM Servo Driver
 * - 3x MG996R/DS3225 servos (continuum arm tendons)
 * - 1x SG90/MG90S servo (gripper)
 * 
 * Wiring:
 * - PCA9685 SDA -> Arduino A4
 * - PCA9685 SCL -> Arduino A5
 * - PCA9685 VCC -> Arduino 5V
 * - PCA9685 GND -> Arduino GND
 * - PCA9685 V+ -> External 6V 10A power supply
 * - Common ground between Arduino and power supply
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channel assignments
#define SERVO_1 0  // Tendon A (Arm)
#define SERVO_2 1  // Tendon B (Arm)
#define SERVO_3 2  // Tendon C (Arm)
#define SERVO_4 3  // Gripper

// Servo pulse length range (microseconds)
#define SERVOMIN  500   // Min pulse length (0 degrees)
#define SERVOMAX  2500  // Max pulse length (180 degrees)

// PCA9685 frequency
#define SERVO_FREQ 50   // Analog servos run at ~50 Hz

// Serial communication
#define BAUD_RATE 115200
#define CMD_BUFFER_SIZE 64

// Current servo positions (degrees)
int currentPositions[4] = {90, 90, 90, 90};  // Start at neutral

// Command buffer
char cmdBuffer[CMD_BUFFER_SIZE];
int bufferIndex = 0;

// Safety limits
#define MIN_ANGLE 0
#define MAX_ANGLE 180

// Timing
unsigned long lastCommandTime = 0;
#define COMMAND_TIMEOUT 5000  // 5 seconds

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Small delay for PCA9685 to stabilize
  delay(100);
  
  // Initialize all servos to neutral position
  for (int i = 0; i < 4; i++) {
    setServoAngle(i, 90);
  }
  
  Serial.println("Arduino Servo Controller Ready");
  Serial.println("Commands: S1:90,S2:90,S3:90,S4:90");
  Serial.println("Angle range: 0-180 degrees");
  Serial.println("================================");
  
  lastCommandTime = millis();
}

void loop() {
  // Check for serial data
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      // End of command
      if (bufferIndex > 0) {
        cmdBuffer[bufferIndex] = '\0';  // Null terminate
        processCommand(cmdBuffer);
        bufferIndex = 0;  // Reset buffer
      }
    } else if (bufferIndex < CMD_BUFFER_SIZE - 1) {
      // Add to buffer
      cmdBuffer[bufferIndex++] = c;
    } else {
      // Buffer overflow - reset
      Serial.println("ERROR: Command too long");
      bufferIndex = 0;
    }
    
    lastCommandTime = millis();
  }
  
  // Safety timeout - return to neutral if no commands
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // Return all servos to neutral position
    for (int i = 0; i < 4; i++) {
      if (currentPositions[i] != 90) {
        setServoAngle(i, 90);
        delay(10);
      }
    }
    lastCommandTime = millis();
  }
  
  delay(10);  // Small delay
}

void processCommand(char* cmd) {
  /*
   * Parse commands in format: S1:90,S2:45,S3:120,S4:0
   * Or single servo: S1:90
   */
  
  Serial.print("RCV: ");
  Serial.println(cmd);
  
  // Parse command
  char* token = strtok(cmd, ",");
  bool success = true;
  int servosUpdated = 0;
  
  while (token != NULL && success) {
    // Parse servo:angle pair
    int servoNum = -1;
    int angle = -1;
    
    if (sscanf(token, "S%d:%d", &servoNum, &angle) == 2) {
      // Valid format
      if (servoNum >= 1 && servoNum <= 4) {
        if (angle >= MIN_ANGLE && angle <= MAX_ANGLE) {
          // Set servo position
          setServoAngle(servoNum - 1, angle);  // Convert to 0-indexed
          servosUpdated++;
        } else {
          Serial.print("ERROR: Angle out of range: ");
          Serial.println(angle);
          success = false;
        }
      } else {
        Serial.print("ERROR: Invalid servo number: ");
        Serial.println(servoNum);
        success = false;
      }
    } else {
      Serial.print("ERROR: Invalid format: ");
      Serial.println(token);
      success = false;
    }
    
    token = strtok(NULL, ",");
  }
  
  if (success && servosUpdated > 0) {
    Serial.print("OK: Updated ");
    Serial.print(servosUpdated);
    Serial.println(" servo(s)");
    
    // Echo current positions
    Serial.print("POS: S1:");
    Serial.print(currentPositions[0]);
    Serial.print(",S2:");
    Serial.print(currentPositions[1]);
    Serial.print(",S3:");
    Serial.print(currentPositions[2]);
    Serial.print(",S4:");
    Serial.println(currentPositions[3]);
  }
}

void setServoAngle(int channel, int angle) {
  /*
   * Set servo angle (0-180 degrees)
   * Converts angle to PWM pulse length
   */
  
  // Clamp angle to valid range
  if (angle < MIN_ANGLE) angle = MIN_ANGLE;
  if (angle > MAX_ANGLE) angle = MAX_ANGLE;
  
  // Convert angle to pulse length
  // Map 0-180 degrees to SERVOMIN-SERVOMAX microseconds
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  // Convert microseconds to PCA9685 pulse value
  // PCA9685 has 12-bit resolution (4096 steps)
  // At 50Hz, each step is ~4.88us
  int pulse = pulseLength * 4096 / 20000;  // 20000us = 20ms = 50Hz period
  
  // Set PWM
  pwm.setPWM(channel, 0, pulse);
  
  // Update current position
  currentPositions[channel] = angle;
}

/*
 * Additional utility commands (can be added to processCommand)
 */

void emergencyStop() {
  /*
   * Emergency stop - return all servos to neutral
   */
  Serial.println("EMERGENCY STOP!");
  for (int i = 0; i < 4; i++) {
    setServoAngle(i, 90);
    delay(20);
  }
}

void testSequence() {
  /*
   * Test all servos in sequence
   */
  Serial.println("Testing servos...");
  
  for (int i = 0; i < 4; i++) {
    Serial.print("Testing servo ");
    Serial.println(i + 1);
    
    // Sweep from 0 to 180
    for (int angle = 0; angle <= 180; angle += 30) {
      setServoAngle(i, angle);
      delay(300);
    }
    
    // Return to neutral
    setServoAngle(i, 90);
    delay(500);
  }
  
  Serial.println("Test complete!");
}

void gripperControl(bool close) {
  /*
   * Simple gripper control
   * true = close, false = open
   */
  if (close) {
    setServoAngle(SERVO_4, 180);  // Close gripper
    Serial.println("Gripper closed");
  } else {
    setServoAngle(SERVO_4, 0);    // Open gripper
    Serial.println("Gripper opened");
  }
}
