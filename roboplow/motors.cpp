// Motor control module implementation
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

void setMotor(int dirPin, int pwmPin, bool forward, int speed) {
	digitalWrite(dirPin, forward ? HIGH : LOW);
	analogWrite(pwmPin, speed);
}

void moveStraight(int speed) {
	setMotor(motor1_dir, motor1_pwm, true, speed);
	setMotor(motor2_dir, motor2_pwm, true, speed);
	setMotor(motor3_dir, motor3_pwm, false, speed);
	setMotor(motor4_dir, motor4_pwm, false, speed);
}

void stopAll() {
	analogWrite(motor1_pwm, 0);
	analogWrite(motor2_pwm, 0);
	analogWrite(motor3_pwm, 0);
	analogWrite(motor4_pwm, 0);
}

void pivotTurn(bool direction, int speed) {
	if (direction == RIGHT) {
		setMotor(motor1_dir, motor1_pwm, false, speed);
		setMotor(motor2_dir, motor2_pwm, false, speed);
		setMotor(motor3_dir, motor3_pwm, false, speed);
		setMotor(motor4_dir, motor4_pwm, false, speed);
	} else {
		setMotor(motor1_dir, motor1_pwm, true, speed);
		setMotor(motor2_dir, motor2_pwm, true, speed);
		setMotor(motor3_dir, motor3_pwm, true, speed);
		setMotor(motor4_dir, motor4_pwm, true, speed);
	}
	delay(1300);
	stopAll();
}
// This is the motors code found in Due_Code.cpp
