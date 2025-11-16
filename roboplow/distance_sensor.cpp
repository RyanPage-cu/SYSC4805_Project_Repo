// Distance sensor module implementation
#include "include/sensors.hpp"
#include "Arduino.h"

// Two Sharp GP2Y0A51SK0F sensors (2–15 cm range)

const int SENSOR_LEFT  = A8;       // Left sensor VOUT
const int SENSOR_RIGHT = A9;       // Right sensor VOUT

/************************************************************
 * Read averaged voltage from the Sharp IR sensor
 ************************************************************/
float readVoltage(int pin) {
    const int SAMPLES = 10;
    long sum = 0;

    for (int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);
        delay(2);  // small sampling delay
    }

    float avg = sum / (float)SAMPLES;

    // Convert analog reading (0–1023) to voltage (0–5 V)
    return avg * (5.0 / 1023.0);
}

/************************************************************
 * Convert Sharp GP2Y0A51SK0F voltage → distance (cm)
 * Approximation formula from sensor curve
 ************************************************************/
float sharpVoltageToDistance(float voltage) {
    if (voltage <= 0.06) return 15.0;  // max range safety

    float dist = 13.0 / (voltage - 0.06);

    // Clamp to sensor's physical range
    if (dist < 2.0) dist = 2.0;
    if (dist > 15.0) dist = 15.0;

    return dist;
}

/************************************************************
 * Read LEFT and RIGHT distance (cm)
 ************************************************************/
void readSharpDistances(float &distLeft, float &distRight) {
    float vLeft  = readVoltage(SENSOR_LEFT);
    float vRight = readVoltage(SENSOR_RIGHT);

    distLeft  = sharpVoltageToDistance(vLeft);
    distRight = sharpVoltageToDistance(vRight);
}

// Example usage in loop():
// float dL, dR;
// readSharpDistances(dL, dR);
// Serial.println(dL);

