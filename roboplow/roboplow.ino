#include <watchdog.h>

#include "include/sensors.hpp"

/************************************************************
 * Watchdog requirement
 ************************************************************/
void watchdogSetup(void) {}

/************************************************************
 * Arduino Setup
 ************************************************************/
void setup() {
  watchdogEnable(4000);

  Serial.begin(115200);
  Serial.println("Robot Initializing...");

  /***** Motor Pins *****/
  motors_init();

  /***** Ultrasonic Sensor Pins *****/
  ultrasonic_init();

  /***** Line Follower Sensor Pins *****/
  line_detection_init();

  // --- Initialize VL53L1X Time-of-Flight sensor ---
  if (!tof_init()) {
    Serial.println("ToF init failed!");
  }

  // --- Initialize IR Obstacle Detection Sensors ---
  ir_init();
}

/************************************************************
 * Arduino Loop
 ************************************************************/
void loop() {
  watchdogReset();

  /***** Read Ultrasonic Distance Value *****/
  float right_DistanceCm = ultrasonic_singleRead_Right();
  Serial.print("Ultrasonic Right: ");
  Serial.println(right_DistanceCm);
  float left_DistanceCm = ultrasonic_singleRead_Left();
  Serial.print("Ultrasonic Left: ");
  Serial.println(left_DistanceCm);

  /***** Read ToF Distance Value *****/
  float front_DistanceCm = tof_readDistance();
  Serial.print("ToF: ");
  Serial.print(front_DistanceCm);
  Serial.println(" cm");

  /***** Read Line Sensors *****/
  int FL, FM, FR, BL, BM, BR;
  readLineSensors(FL, FM, FR, BL, BM, BR);
  Serial.print("Front Line Sensors 1 - L:");
  Serial.print(FL);
  Serial.print(" M:");
  Serial.print(FM);
  Serial.print(" R:");
  Serial.println(FR);
  Serial.print("Back Line Sensors 2 - L:");
  Serial.print(BL);
  Serial.print(" M:");
  Serial.print(BM);
  Serial.print(" R:");
  Serial.println(BR);

  /***** Read IR Obstacle Detection Sensors *****/
  bool obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
  bool obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);
  Serial.print("IR Front Left Obstacle: ");
  Serial.println(obstacleFrontLeft ? "Yes" : "No");
  Serial.print("IR Front Right Obstacle: ");
  Serial.println(obstacleFrontRight ? "Yes" : "No");

  if (front_DistanceCm < 30) {
    // Obstacle detected in front
    Serial.println("Obstacle detected in front! Stopping.");
    stopAll();
    delay(1000);

    // Decide pivot direction based on side distances
    if (right_DistanceCm > left_DistanceCm) {
      if (!obstacleFrontRight) {   // check if we can rotate right
        if (FR == 0 || FM == 0) {  // check if there's no black line in front
          Serial.println("Pivoting Right");
          pivotTurn(RIGHT, 150);
        } else {
          Serial.println("Can't pivot right.");  // try to check if you can
                                                 // rotate the robot 180 deg
        }
      } else {
        // There's space on the right side, but obstacle on the front-right IR
        // sensor is blocking us from taking a turn Try to figure out how to
        // move back a bit, and then try to take a turn
      }
    } else if (right_DistanceCm <= left_DistanceCm) {
      if (!obstacleFrontLeft) {    // check if we can rotate left
        if (FL == 0 || FM == 0) {  // check if there's no black line in front
          Serial.println("Pivoting Left");
          pivotTurn(LEFT, 150);
        } else {
          Serial.println("Can't pivot left.");  // try to check if you can
                                                // rotate the robot 180 deg
        }
      } else {
        // There's space on the left side, but obstacle on the front-left IR
        // sensor is blocking us from taking a turn Try to figure out how to
        // move back a bit, and then try to take a turn
      }
    }
  } else {
    stepForward(180, 500);  // No obstacle, move forward
  }

  /*
  if(front_DistanceCm > 0 && front_DistanceCm < 30) {
      Serial.println("Obstacle detected! Stopping.");
      stopAll();
      delay(1000);
      if(right_DistanceCm > left_DistanceCm) {
          Serial.println("Pivoting Right");
          pivotTurn(RIGHT, 150);
      } else {
          Serial.println("Pivoting Left");
          pivotTurn(LEFT, 150);
      }
  }

  if(right_DistanceCm > 0 && right_DistanceCm < 20) {
      stopAll();
      delay(1000);
      if(front_DistanceCm < 30){
          Serial.println("Right obstacle detected and Front! Pivoting Left.");
          pivotTurn(LEFT, 150);
      }else{
          Serial.println("Right obstacle detected and nothing in Front! Move
  Straight.");
      }
  }
  if(left_DistanceCm > 0 && left_DistanceCm < 20) {
      stopAll();
      delay(1000);
      if(front_DistanceCm < 30){
          Serial.println("Left obstacle detected and Front! Pivoting Right.");
          pivotTurn(RIGHT, 150);
      }else{
          Serial.println("Left obstacle detected and nothing in Front! Stay
  Straight.");
      }
  }

  if(FL == 0 || FM == 0 || FR == 0){
      Serial.println("Line detected on Front Sensors! Stopping.");
      stopAll();
      if(FL == 0 && FR == 1 && FM == 1 || (FL == 0 && FM == 0 && FR == 1)){
          Serial.println("Left Front Sensor on Line! Pivoting Right.");
          stepForward(-180, 500);
          delay(1000);
          pivotTurn(RIGHT, 150);
      }else if(FR == 0 && FL == 1 && FM == 1 || (FR == 0 && FM == 0 && FL ==
  1)){ Serial.println("Right Front Sensor on Line! Pivoting Left.");
          stepForward(-180, 500);
          pivotTurn(LEFT, 150);
      }else{
          Serial.println("Middle Front Sensor on Line! Reversing.");
          stepForward(-180, 1000);
          if(right_DistanceCm > left_DistanceCm) {
              Serial.println("Pivoting Right");
              pivotTurn(RIGHT, 150);
          } else {
              Serial.println("Pivoting Left");
              pivotTurn(LEFT, 150);
          }
   }
      delay(1000);
  }

  stepForward(180, 1000);
  */
  delay(1000);
}
