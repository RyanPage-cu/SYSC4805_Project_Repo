#include "include/sensors.hpp"
#include <watchdog.h>

void setup() {
	Serial.begin(115200);

	// Motor Pins
	pinMode(motor1_dir, OUTPUT);
	pinMode(motor2_dir, OUTPUT);
	pinMode(motor3_dir, OUTPUT);
	pinMode(motor4_dir, OUTPUT);

	// Ultrasonic Sensor Pins
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);

	// Line Follower Sensor Pins
	pinMode(LFS_L1, INPUT);
	pinMode(LFS_M1, INPUT);
	pinMode(LFS_R1, INPUT);

	pinMode(LFS_L2, INPUT);
	pinMode(LFS_M2, INPUT);
	pinMode(LFS_R2, INPUT);

	// Enable watchdog timer with 2000ms timeout
	watchdogEnable(2000);

	Serial.println("Robot Initialized with Analog Line Detectors");
}

void loop() {
	// Reset watchdog timer
	watchdogReset();

	ultrasonic(); // Check obstacle

	int leftState1, middleState1, rightState1, leftState2, middleState2, rightState2;
	readLineSensors(leftState1, middleState1, rightState1, leftState2, middleState2, rightState2);

	// Simple line-following logic
	if(leftState1 == HIGH) {  // Black detected on left sensor, turn away to the right
		stopAll();
		pivotTurn(RIGHT, 180);
	} 
	else if(rightState1 == HIGH) { // Black detected on right sensor, turn away to the left
		stopAll();
		pivotTurn(LEFT, 180);
	} 
	else {
		moveStraight(180);
	}

	delay(20); // Adjust loop speed
}
