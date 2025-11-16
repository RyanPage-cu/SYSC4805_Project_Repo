// Ultrasonic sensor module implementation

#include "include/sensors.hpp"
#include "Arduino.h"

const int TRIG_PIN = 43;   // Trigger pin
const int ECHO_PIN = A6;   // Echo pin
float distanceCm;

void ultrasonic_readDistance() {
	long duration;
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	duration = pulseIn(ECHO_PIN, HIGH);
	distanceCm = (duration * 0.034) / 2;
}
