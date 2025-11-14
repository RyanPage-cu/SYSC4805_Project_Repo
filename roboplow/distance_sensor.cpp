// Distance sensor module implementation
#include "include/sensors.hpp"
#include "Arduino.h"

// Two Sharp GP2Y0A51SK0F sensors (2–15 cm range)
// Each sensor reports independently when its voltage > 0.72 V (~object closer than 8 cm)

const int SENSOR_LEFT  = A1;       // Left sensor VOUT
const int SENSOR_RIGHT = A2;       // Right sensor VOUT
const float THRESHOLD_V = 0.72;    // ~8 cm boundary voltage

// Function to read average voltage from analog pin
float readVoltage(int pin) {
	const int SAMPLES = 10;
	long sum = 0;
	for (int i = 0; i < SAMPLES; i++) {
		sum += analogRead(pin);
		delay(2);
	}
	return (sum / (float)SAMPLES) * (5.0 / 1023.0);
}

// Example usage (to be called from main.ino):
// float vLeft  = readVoltage(SENSOR_LEFT);
// float vRight = readVoltage(SENSOR_RIGHT);
// if (vLeft > THRESHOLD_V) { /* object detected */ }
// This is the distance sensor code found in distance_sensor.ino
