// === sensor_manager.hpp ===
// This header defines pin mappings, thresholds, and function prototypes for all sensors.
// Each sensor module should include its own implementation file.
#include stings.hpp
#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include <Arduino.h> // provides uint32_t and Arduino types

// --- Motor Control (to be moved to motors.cpp) ---
extern const int motor1_dir;
extern const int motor1_pwm;
extern const int motor2_dir;
extern const int motor2_pwm;
extern const int motor3_dir;
extern const int motor3_pwm;
extern const int motor4_dir;
extern const int motor4_pwm;
extern const bool RIGHT;
extern const bool LEFT;
void motors_init(); // Implementation in motors.cpp
void setMotor(int dirPin, int pwmPin, bool forward, int speed); // Implementation in motors.cpp
void moveStraight(int speed = 180); // Implementation in motors.cpp
void stopAll(); // Implementation in motors.cpp
void pivotTurn(bool direction, int speed = 180); // Implementation in motors.cpp
void stepForward(int speed, unsigned long duration_ms); // Implementation in motors.cpp

// --- Ultrasonic Sensor (to be moved to ultrasonic.cpp) ---
extern const int TRIG_PIN0;
extern const int ECHO_PIN0;
extern const int TRIG_PIN1;
extern const int ECHO_PIN1;
void ultrasonic_init(); // Implementation in ultrasonic.cpp
float ultrasonic_singleRead_Left();  // Implementation in ultrasonic.cpp
float ultrasonic_singleRead_Right();  // Implementation in ultrasonic.cpp
float ultrasonic_readDistance_Left(); // Implementation in ultrasonic.cpp
float ultrasonic_readDistance_Right(); // Implementation in ultrasonic.cpp

// --- Line Follower Sensors (to be moved to line_follower.cpp) ---
extern const int LFS_R1;
extern const int LFS_M1;
extern const int LFS_L1;
extern const int LFS_R2;
extern const int LFS_M2;
extern const int LFS_L2;
extern const int LINE_THRESHOLD;
void line_detection_init(); // Implementation in line_follower.cpp
void readLineSensors(int &leftState1, int &middleState1, int &rightState1, int &leftState2, int &middleState2, int &rightState2); // Implementation in line_follower.cpp

// --- Time of Flight Sensor (VL53L1X) ---
bool tof_init();                             // initialize sensor, returns true on success
float tof_readDistance();                    // read distance in cm, returns -1.0 on timeout/error    

// --- IR Obstacle Detection Sensor ---
extern const int IR_SENSOR_FR;    // Digital input from VMA330 OUT pin
extern const int IR_SENSOR_FL;    // Digital input from VMA330 OUT pin
void ir_init(); // initialize IR sensors
bool ir_obstacleDetected(int IR_SENSOR_PIN);       // returns true if obstacle detected, prints status

// --- Magnetometer (LIS3MDL) ---
extern char heading;
extern static char initialHeading;
extern static char startingSide;
void magnetometer_init(); // initialize magnetometer
char read_heading();     // read heading in degrees

#endif
