#include "include/sensor_manager.hpp"
#include "Arduino.h"

// Line Follower Sensor #1 Pins (analog)
const int LFS_R1 = A2;   // Left sensor
const int LFS_M1 = A0;   // Middle sensor
const int LFS_L1 = A1;   // Right sensor
// Line Follower Sensor #2 Pins (analog)
const int LFS_R2 = A3;   // Left sensor
const int LFS_M2 = A4;   // Middle sensor
const int LFS_L2 = A5;   // Right sensor

const int LINE_THRESHOLD = 880;

void line_detection_init() {
    pinMode(LFS_L1, INPUT);
    pinMode(LFS_M1, INPUT);
    pinMode(LFS_R1, INPUT);

		/*
    pinMode(LFS_L2, INPUT);
    pinMode(LFS_M2, INPUT);
    pinMode(LFS_R2, INPUT);
		*/
}

void readLineSensors(int &leftState1, int &middleState1, int &rightState1) {
	int leftVal1 = analogRead(LFS_L1);
	int middleVal1 = analogRead(LFS_M1);
	int rightVal1 = analogRead(LFS_R1);

	/*
	int leftVal2 = analogRead(LFS_L2);
	int middleVal2 = analogRead(LFS_M2);
	int rightVal2 = analogRead(LFS_R2);
	*/

	// Convert to HIGH if black detected, LOW if white
	leftState1 = (leftVal1 > LINE_THRESHOLD) ? HIGH : LOW;
	middleState1 = (middleVal1 > LINE_THRESHOLD) ? HIGH : LOW;
	rightState1 = (rightVal1 > LINE_THRESHOLD) ? HIGH : LOW;

}
