#include "include/sensors.hpp"
#include "Arduino.h"

const int motor1_dir = 53;
const int motor1_pwm = 52;
const int motor2_dir = 51;
const int motor2_pwm = 50;
const int motor3_dir = 49;
const int motor3_pwm = 48;
const int motor4_dir = 47;
const int motor4_pwm = 46;
const bool RIGHT = true;
const bool LEFT = false;

void motors_init() {
    pinMode(motor1_dir, OUTPUT);
    pinMode(motor2_dir, OUTPUT);
    pinMode(motor3_dir, OUTPUT);
    pinMode(motor4_dir, OUTPUT);    
}

void setMotor(int dirPin, int pwmPin, bool forward, int speed) {
	digitalWrite(dirPin, forward ? HIGH : LOW);
	analogWrite(pwmPin, speed);
}

void moveForward(int speed) {
	setMotor(motor1_dir, motor1_pwm, true, speed);
	setMotor(motor2_dir, motor2_pwm, true, speed);
	setMotor(motor3_dir, motor3_pwm, false, speed);
	setMotor(motor4_dir, motor4_pwm, false, speed);
}

void moveBackward(int speed) {
	setMotor(motor1_dir, motor1_pwm, false, speed);
	setMotor(motor2_dir, motor2_pwm, false, speed);
	setMotor(motor3_dir, motor3_pwm, true, speed);
	setMotor(motor4_dir, motor4_pwm, true, speed);
}

void stopAll() {
	analogWrite(motor1_pwm, 0);
	analogWrite(motor2_pwm, 0);
	analogWrite(motor3_pwm, 0);
	analogWrite(motor4_pwm, 0);
}

// Non-blocking pivot API
static unsigned long pivotEndTime = 0;
static bool pivotActive = false;
static bool pivotDirection = RIGHT;
static int pivotSpeed = 0;

void startPivot(bool direction, int speed, unsigned long duration_ms) {
    speed = constrain(speed, 0, 255);
    pivotDirection = direction;
    pivotSpeed = speed;
    pivotEndTime = millis() + duration_ms;
    pivotActive = true;

    if (pivotDirection == RIGHT) {
        setMotor(motor1_dir, motor1_pwm, false, pivotSpeed);
        setMotor(motor2_dir, motor2_pwm, false, pivotSpeed);
        setMotor(motor3_dir, motor3_pwm, false, pivotSpeed);
        setMotor(motor4_dir, motor4_pwm, false, pivotSpeed);
    } else {
        setMotor(motor1_dir, motor1_pwm, true, pivotSpeed);
        setMotor(motor2_dir, motor2_pwm, true, pivotSpeed);
        setMotor(motor3_dir, motor3_pwm, true, pivotSpeed);
        setMotor(motor4_dir, motor4_pwm, true, pivotSpeed);
    }
}

void updatePivot() {
    if (!pivotActive) return;
    if ((long)(millis() - pivotEndTime) >= 0) { // handles wrap-around
        stopAll();
        pivotActive = false;
    }
}

void stopPivot() {
    stopAll();
    pivotActive = false;
}

// Blocking wrapper that preserves existing behaviour (default 1500 ms)
void pivotTurn(bool direction, int speed) {
    startPivot(direction, speed, 1200);
    while (pivotActive) {
        updatePivot();
        delay(5); // tiny sleep to avoid busy loop; keeps responsiveness
    }
}

void stepForward(int speed, unsigned long duration_ms) {
    if(speed < 0) {
        moveBackward(-1 * speed);
    } else {
        moveForward(speed);
    }
    delay(duration_ms);
    stopAll();
}
